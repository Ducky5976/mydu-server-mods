using Orleans;
using System;
using System.IO;
using System.Linq;
using System.Net.Http;
using System.Diagnostics;
using System.Threading.Tasks;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Reflection;
using Backend;
using Backend.Business;
using Backend.Database;
using NQutils.Config;
using Backend.Construct;
using Backend.Storage;
using Backend.Scenegraph;
using NQ;
using NQ.RDMS;
using NQ.Interfaces;
using NQ.Visibility;
using NQ.Grains.Core;
using NQutils;
using NQutils.Exceptions;
using NQutils.Net;
using NQutils.Serialization;
using NQutils.Sql;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Logging;
using Newtonsoft.Json;
using MathNet.Spatial.Euclidean;
using MathNet.Numerics;

public class DtoPath
{
    public List<double> origin;
    public List<double> destination;
}

// Stores waypoints and forward path queries for a construct
public class Building
{
    public readonly ulong constructId;
    public readonly IServiceProvider isp;
    public List<NQ.Vec3> waypoints = new();
    public HttpClient httpClient;
    public ulong targetPlayerId;
    public ulong nextId = 1000000;
    static public string navUrl = "http://192.168.1.15:8879"; // ADJUST ME
    
    public Building(ulong cid, IServiceProvider isp)
    {
        constructId = cid;
        this.isp = isp;
        httpClient = isp.GetRequiredService<IHttpClientFactory>().CreateClient();
    }
    public async Task Initialize()
    {
        var orleans = isp.GetRequiredService<IClusterClient>();
        var elems = await orleans.GetConstructElementsGrain(constructId).GetVisibleAt(0);
        foreach (var el in elems.elements)
        {
            if (el.elementType == 2012928469) // Pressure plate
                waypoints.Add(el.position);
        }
    }
    public async Task<List<NQ.Vec3>> QueryPath(DtoPath query)
    {
        var path = await httpClient.Post<List<List<double>>>($"{navUrl}/navigation/path/{constructId}", query, binary: false);
        var res = new List<NQ.Vec3>();
        foreach(var p in path)
        {
            res.Add(new NQ.Vec3 { x = p[0], y = p[1], z = p[2]});
        }
        return res;
    }
}

// NPC wandering around a construct
public class Character
{
    public Building building;
    public readonly ulong playerId;
    public List<NQ.Vec3> waypoints = new();
    public int waypointIndex;
    public NQ.Vec3 position;
    public DateTime lastRay;
    public bool inSight;
    public Random rnd;
    public Character(ulong pid, Building b)
    {
        building = b;
        playerId = pid;
        rnd = new();
        position = building.waypoints[rnd.Next(0, building.waypoints.Count)];
        lastRay = DateTime.Now;
        _ = Task.Factory.StartNew(async () => {
                while (true)
                {
                    try
                    {
                        await PositionLoop();
                    }
                    catch (Exception e)
                    {
                        building.isp.GetRequiredService<ILogger<Character>>()
                           .LogError(e, "bronking in position loop");
                    }
                }
        });
    }
    protected async Task PositionLoop()
    {
        while (true)
        {
            if (waypointIndex >= waypoints.Count)
            {
                var target = building.waypoints[rnd.Next(0, building.waypoints.Count)];
                waypoints = await building.QueryPath(new DtoPath
                    {
                        origin = new List<double>{ position.x, position.y, position.z},
                        destination = new List<double>{target.x, target.y, target.z},
                    });
                waypointIndex = 1;
            }
            var direction = waypoints[waypointIndex] - position;
            if (direction.Norm() < 0.4)
            {
                waypointIndex += 1;
                continue;
            }
            direction = direction * (0.25 / direction.Norm()); // 1m/s
            var next = position + direction;
            position = next;
            var loc = new SimpleLocation
            {
                ConstructId = building.constructId,
                Position = position,
            };
            var rot = MyDuMod.ToQuatFromDirection(direction, 0, false);
            var pu = new PlayerUpdate
            {
                playerId = playerId,
                constructId = building.constructId,
                position = position,
                rotation = rot,
                velocity = direction * 10.0,
                time = TimePoint.Now(),
            };
            var op = new NQutils.Messages.PlayerUpdate(pu);
            await building.isp.GetRequiredService<NQ.Visibility.Internal.InternalClient>()
                .PublishGenericEventAsync(new EventLocation
                    {
                        Event = NQutils.Serialization.Grpc.MakeEvent(op),
                        Location = loc,
                        VisibilityDistance = 1000,
                    });

                
            var now = DateTime.Now;
            if (now - lastRay > TimeSpan.FromSeconds(1) && building.targetPlayerId != 0)
            {
                lastRay = now;
                 await building.isp.GetRequiredService<IPub>().NotifyTopic(Topics.PlayerNotifications(building.targetPlayerId),
                    new NQutils.Messages.ModTriggerHudEventRequest(new ModTriggerHudEvent
                        {
                            eventName = "modinjectjs",
                            eventPayload = $"window.check({playerId});",
                        }));
            }
            await Task.Delay(TimeSpan.FromMilliseconds(100));
        }
    }
    // receives a raycast info from the player
    public async Task receiveRaycast(Raycast rc)
    {
        bool newVisible = rc.playerId == playerId;
        await receiveCheck(newVisible);
    }
    public async Task receiveCheck(bool newVisible)
    { // Send a weapon FX when player becomes visible
        if (newVisible == inSight)
            return;
        await building.isp.GetRequiredService<IClusterClient>()
            .GetChatGrain(playerId).SendMessage(new NQ.MessageContent
                {
                    channel = new MessageChannel
                    {
                        channel = MessageChannelType.HELP,
                    },
                    message = newVisible ? "Target in sight": "target lost",
                });
        if (newVisible)
        {
            NQ.Vec3 srcRel = position + new NQ.Vec3{z=1.25};
            NQ.RelativeLocation srcRLoc = new NQ.RelativeLocation
                {
                    constructId = building.constructId,
                    position = srcRel,
                    rotation = NQ.Quat.Identity,
                };
            
            var srcALoc = await building.isp.GetRequiredService<IScenegraph>().ResolveWorldLocation(srcRLoc);

            var tg = await building.isp.GetRequiredService<IScenegraph>().GetPlayerWorldPosition(building.targetPlayerId);
            tg.Item1.position = tg.Item1.position + new NQ.Vec3{z= 1.25}; // ********* NO, IF CONSTRUCT ROTATED FAILS
            tg.Item2.position = tg.Item2.position + new NQ.Vec3{z= 1.25}; // ********* NO, IF CONSTRUCT ROTATED FAILS
                
            var bank = building.isp.GetRequiredService<IGameplayBank>();
            var ws = new WeaponShot
            {
                id = building.nextId++,
                originConstructId = building.constructId,
                weaponId = 0,
                // Picking laser because it as the shortest FX (data/resources_generated/effects.nqdef)
                weaponType = bank.GetDefinition("WeaponLaserExtraSmallAgile3").Id,
                ammoType = bank.GetDefinition("AmmoLaserExtraSmallThermicAdvancedAgile").Id,
                //weaponType = bank.GetDefinition("WeaponMissileLarge1").Id,
                //ammoType = bank.GetDefinition("AmmoMissileLargeAntimatterAdvancedAgile").Id,
                //weaponType = bank.GetDefinition("WeaponRailgunExtraSmallAgile3").Id,
                //ammoType = bank.GetDefinition("AmmoRailgunExtraSmallAntimatterAdvancedAgile").Id,
                originPositionLocal = srcRel,
                originPositionWorld = srcALoc.position,
                targetConstructId = building.constructId,
                impactPositionLocal = tg.Item1.position,
                impactPositionWorld = tg.Item2.position,
            };
            // FX
            var op = new NQutils.Messages.WeaponShot(ws);
            var ev = NQutils.Serialization.Grpc.MakeEvent(op);
            var loc = new SimpleLocation
            {
                ConstructId = 0, // action.constructId,
                Position = tg.Item2.position, // tr.position,
            };
            await building.isp.GetRequiredService<NQ.Visibility.Internal.InternalClient>().PublishGenericEventAsync(new EventLocation
                {
                    Event = ev,
                    Location = loc,
                    VisibilityDistance =  4000,
                });
        }
        inSight = newVisible;
    }
}
public class Raycast
{ // client raycast payload struct
    public ulong playerId;
    public ulong constructId;
    public ulong elementId;
    public List<double> impactPoint;
    public List<double> impactNormal;
};

public class MyDuMod: IMod
{
    private IServiceProvider isp;
    private IClusterClient orleans;
    private ILogger logger;
    private IGameplayBank bank;
    private NQ.Visibility.Internal.InternalClient client;
    private IScenegraph scenegraph;
    private IPub pub;
    private Dictionary<ulong, DateTime> debouncer = new();
    private Dictionary<ulong, bool> registered = new();
    private ulong nextId = 99999;

    private Building building;
    private List<Character> npcs = new();
    public string GetName()
    {
        return "Weapon";
    }
    public async Task Initialize(IServiceProvider isp)
    {
        this.isp = isp;
        this.orleans = isp.GetRequiredService<IClusterClient>();
        this.logger = isp.GetRequiredService<ILogger<MyDuMod>>();
        this.bank = isp.GetRequiredService<IGameplayBank>();
        this.client = isp.GetRequiredService<NQ.Visibility.Internal.InternalClient>();
        this.scenegraph = isp.GetRequiredService<IScenegraph>();
        this.pub = isp.GetRequiredService<IPub>();
        // Adjust this with existing construct id and player IDs
        building = new Building(1000215, isp);
        await building.Initialize();
        npcs.Add(new Character(10002, building));
        npcs.Add(new Character(10003, building));
        npcs.Add(new Character(10004, building));
        npcs.Add(new Character(10005, building));
        npcs.Add(new Character(10006, building));
    }
    public Task<ModInfo> GetModInfoFor(ulong playerId, bool admin)
    {
        registered.Remove(playerId);
        return Task.FromResult<ModInfo>(new ModInfo
            {
                name = GetName(),
                actions = new List<ModActionDefinition>
                {
                    new ModActionDefinition
                    {
                        id = 1000,
                        label = "super shot",
                        context = ModActionContext.Global,
                    },
                    new ModActionDefinition
                    {
                        id = 10003,
                        label = "player info",
                        context = ModActionContext.Global,
                    },
                    new ModActionDefinition
                    {
                        id = 10004,
                        label = "Set repulsor point",
                        context = ModActionContext.Global,
                    },
                    new ModActionDefinition
                    {
                        id = 20000,
                        label = "Install hacker",
                        context = ModActionContext.Element,
                    },
                    new ModActionDefinition
                    {
                        id = 1,
                        label = "Register target",
                        context = ModActionContext.Global,
                    },
                    new ModActionDefinition
                    {
                        id = 6,
                        label = "Enable minimap",
                        context = ModActionContext.Global,
                    },
                }
            });
    }
    public static NQ.Quat ToQuatFromAngle(NQ.Vec3 v, double angle )
    {
        var q =  System.Numerics.Quaternion.CreateFromAxisAngle(
            new System.Numerics.Vector3((float)v.x, (float)v.y, (float)v.z), (float)angle);
        return new NQ.Quat { x=q.X, y = q.Y, z=q.Z, w=q.W};
    }
    public static NQ.Quat ToQuatFromDirection(NQ.Vec3 v, double roll = 0, bool fromZ = true)
    {
        NQ.Vec3 fdir = fromZ ? new NQ.Vec3{ z = 1} : new NQ.Vec3{ y = 1};
        var rotaxis = fdir.Cross(v.Normalized());
        var angle = Math.Atan2(rotaxis.Norm(), fdir.Dot(v));
        return ToQuatFromAngle(rotaxis.Normalized(), angle);
    }
    public static NQ.Vec3 ToEulerAngles(NQ.Quat q)
    {
        var e = ((Quaternion)q).ToEulerAngles();
        return new NQ.Vec3
        {
            x = e.Alpha.Radians,
            y = e.Beta.Radians,
            z = e.Gamma.Radians,
        }; // or is it?
    }
    // Blink the hacker and at the end of the countdown open target door
    public async Task RunHacker(ulong constructId, ulong hackerElementId, ulong targetElementId, double delay)
    {
        var ceg = orleans.GetConstructElementsGrain(constructId);
        
        var end = DateTime.Now + TimeSpan.FromMilliseconds(delay * 1000.0);
        var state = false;
        while (true)
        {
            var now = DateTime.Now;
            var remain = (end-now).TotalSeconds;
            if (remain < 0)
                break;
            var tickDuration = remain / 10.0;
            state = !state;
            await ceg.UpdateElementProperty(new ElementPropertyUpdate
                {
                    constructId = constructId,
                    elementId = hackerElementId,
                    name = "button_on",
                    value = new PropertyValue(state),
                    timePoint = TimePoint.Now(),
                });
            await Task.Delay(TimeSpan.FromMilliseconds(tickDuration * 1000));
        }
        // trigger hack
        await ceg.UpdateElementProperty(new ElementPropertyUpdate
                {
                    constructId = constructId,
                    elementId = targetElementId,
                    name = "element_state",
                    value = new PropertyValue(true),
                    timePoint = TimePoint.Now(),
                });
        await ceg.DeleteElement(hackerElementId, null, true);
    }
    // Add a hacker element with proper orientation according to impact normal
    public async Task AddHacker(ulong playerId, ModAction action)
    {
        var hitinfo = JsonConvert.DeserializeObject<Raycast>(action.payload);
        var normal = new NQ.Vec3 { x = hitinfo.impactNormal[0], y = hitinfo.impactNormal[1], z=hitinfo.impactNormal[2]};
        var ta = new NQ.RelativeLocation
        {
            position = new NQ.Vec3 { x = hitinfo.impactPoint[0], y = hitinfo.impactPoint[1], z = hitinfo.impactPoint[2]},
            rotation = ToQuatFromDirection(normal.Normalized(), 0).Normalized(),
        };
        var tr = await scenegraph.ResolveRelativeLocation(ta, hitinfo.constructId);
        var ceg = orleans.GetConstructElementsGrain(hitinfo.constructId);
        var el = await ceg.GetElement(hitinfo.elementId);
        var q = (Quaternion)el.rotation;
        var qr = (Quaternion)ToQuatFromAngle(new NQ.Vec3{ x = 1}, 3.14159 / 2.0); //y=noop?
        var qq = q * qr;
        var eToC = ((Quaternion)el.rotation).Rotate((Vector3D)normal);
        var lrot = ToQuatFromDirection(eToC, 0);
        logger.LogInformation("Construct {cid} ap {apos} rp {rpos} normalworld {normal} normallocal q={qrot}  xyz={eulerrot}", hitinfo.constructId, ta.position, tr.position, normal, tr.rotation, ToEulerAngles(tr.rotation));
        logger.LogInformation("Math says {q} {qr} {qq}", el.rotation, (NQ.Quat)qr, (NQ.Quat)qq);
        var ei = new ElementInfo
        {
            constructId = hitinfo.constructId,
            elementType = 1550904282, // ManualButtonSmall
            position = tr.position,
            rotation = tr.rotation,
            //rotation = (NQ.Quat)qq,
        };
        var hei = await ceg.AddElement(2, ei, new());
        _ = RunHacker(hitinfo.constructId, hei.elementId, hitinfo.elementId, 8.0);
    }

    // Basic damaging shot
    public async Task Shot(ulong playerId, ModAction action)
    {
        var sg = isp.GetRequiredService<IScenegraph>();
        var pub = isp.GetRequiredService<IPub>();
        var bank = isp.GetRequiredService<IGameplayBank>();
        var (fr, fa) = await sg.GetPlayerWorldPosition(playerId);
        var tap = JsonConvert.DeserializeObject<List<double>>(action.payload);
        var ta = new NQ.RelativeLocation
        {
            constructId = 0,
            position = new NQ.Vec3{x = tap[0], y = tap[1], z = tap[2]},
            rotation = NQ.Quat.Identity,
        };
        var tr = await sg.ResolveRelativeLocation(ta, action.constructId);
        tr.constructId = action.constructId;
        //this.logger.LogWarning($"shot targetting {action.payload} {tap[0]} {tap[1]} {tap[2]} resolved to {tr.position.x} {tr.position.y} {tr.position.z}");
        var ws = new WeaponShot
        {
            id = nextId++,
            originConstructId = action.constructId,
            weaponId = 0,
            // Picking laser because it as the shortest FX (data/resources_generated/effects.nqdef)
            weaponType = bank.GetDefinition("WeaponLaserExtraSmallAgile3").Id,
            ammoType = bank.GetDefinition("AmmoLaserExtraSmallThermicAdvancedAgile").Id,
            //weaponType = bank.GetDefinition("WeaponMissileLarge1").Id,
            //ammoType = bank.GetDefinition("AmmoMissileLargeAntimatterAdvancedAgile").Id,
            //weaponType = bank.GetDefinition("WeaponRailgunExtraSmallAgile3").Id,
            //ammoType = bank.GetDefinition("AmmoRailgunExtraSmallAntimatterAdvancedAgile").Id,
            originPositionLocal = fr.position,
            originPositionWorld = fa.position,
            targetConstructId = action.constructId,
            impactPositionLocal = tr.position,
            impactPositionWorld = ta.position,
        };
        // FX
        var op = new NQutils.Messages.WeaponShot(ws);
        var ev = NQutils.Serialization.Grpc.MakeEvent(op);
        var loc = new SimpleLocation
        {
            ConstructId = 0, // action.constructId,
            Position = ta.position, // tr.position,
        };
        await client.PublishGenericEventAsync(new EventLocation
            {
                Event = ev,
                Location = loc,
                VisibilityDistance =  4000,
            });
        // debug
        /*`await pub.NotifyTopic(Topics.PlayerNotifications(playerId),
            new NQutils.Messages.WeaponShot(ws)
            );*/
        //damage
        var wf = new WeaponFire
        {
            playerId = playerId,
            weaponId = 0,
            constructId = action.constructId,
            seatId = 0,
            targetId = action.constructId,
            impactElementId = action.elementId,
            impactElementType = bank.GetDefinition("Chair").Id,
            impactPoint = tr.position,
            bboxCenterLocal = tr.position,
            bboxSizeLocal = new NQ.Vec3 { x = 16, y = 16, z = 16},
        };
        // That would tell us if player is caught in explosion, but
        // is somehow super-slow.
        //var killablePlayers = await orleans.GetConstructGrain(state.cid).GetKillablePlayerListAndPosition();
        var killablePlayers = new List<(NQ.PlayerId, NQ.Vec3)>();
        var deathInfoPvp = new PlayerDeathInfoPvPData
        {
            weaponId = 0,
            weaponTypeId = bank.GetDefinition("WeaponRailgunExtraSmallAgile3").Id,
            constructId = action.constructId,
            constructName = "unnamed",
            playerId = playerId,
            playerName = "unnamed",
            ownerId = new NQ.EntityId { playerId = playerId},
        };
        // fire voxel op async to not freeze stuff
        _ = Task.Factory.StartNew( async () => {
                var voxelResult = await orleans.GetDirectServiceGrain().MakeVoxelDamages(wf, bank.GetBaseObject<NQutils.Def.Ammo>(bank.GetDefinition("AmmoMissileLargeAntimatterAdvancedAgile").Id), 2000, killablePlayers);
                if (voxelResult.damageOutput != null)
                {
                    await orleans.GetConstructDamageElementsGrain(action.constructId).ApplyPvpElementsDamage(voxelResult.damageOutput.elements, deathInfoPvp);
                }
        });
    }
    public string Escape(string s)
    {
        return s.Replace("\"", "\\\"").Replace("\n", "\\\n");
    }
    // Show a minimap of the voxels (spliced at a fixed altitude)
    // Update the minimap with avatar position and orientation
    // Avatar id list hardcoded below for now
    public async Task InjectMinimap(ulong playerId)
    {
        var loc = await scenegraph.GetPlayerLocation(playerId);
        var z = loc.position.z + 0.56789;
        var httpClient = isp.GetRequiredService<IHttpClientFactory>().CreateClient();
        var svg = await httpClient.PostRaw($"{Building.navUrl}/navigation/map/{loc.constructId}/{z}", "");
        var js = @"
        if (!window.minimap) {
          window.quaternion_mult = function(q,r) {
            return [r[0]*q[0]-r[1]*q[1]-r[2]*q[2]-r[3]*q[3],
                r[0]*q[1]+r[1]*q[0]-r[2]*q[3]+r[3]*q[2],
                r[0]*q[2]+r[1]*q[3]+r[2]*q[0]-r[3]*q[1],
                r[0]*q[3]-r[1]*q[2]+r[2]*q[1]+r[3]*q[0]]
          }
          window.point_rotation_by_quaternion = function(point,q) {
              r = [0, point[0], point[1], point[2]];
              q_conj = [q[0],-1*q[1],-1*q[2],-1*q[3]]
              var res = window.quaternion_mult(window.quaternion_mult(q,r),q_conj);
              return [res[1],res[2],res[3]];
          }
            window.minimap = createElement(document.body, ""div"");
            window.minimap.style.width = ""200px"";
            window.minimap.style.height = ""200px"";
            window.minimap.style.position = ""absolute"";
            window.minimap.style.left = (window.innerWidth - 200) + ""px"";
            window.minimap.style.top = ""100px""
            window.minimap.style.overflow = ""hidden"";
            window.minimap.style.backgroundColor = ""white"";
            window.minimap.style.zIndex = 100000;
            console.log(""pe "" + window.minimap.style.pointerEvents);
            window.minimap.style.pointerEvents = ""all"";
            var mmSize = 200;
            window.minimap.addEventListener(""click"", function(ev) {
                    console.log(""click click"");
                    mmSize *= 0.75;
                    if (mmSize < 50)
                        mmSize = 200;
                    console.log(mmSize);
            });
            window.minimap.addEventListener(""wheel"", function(ev) {
                    console.log(""wheel"");
                    if (ev.deltaY > 0)
                        mmSize *= 1.2;
                    else
                        mmSize *= 0.85;
            });
            window.setInterval(() => {
                    var map = document.getElementById(""minimap"");
                    map.setAttribute(""width"", 200);
                    map.setAttribute(""height"", 200);
                    /*
                    var w = map.getAttribute(""width"");
                    var h = map.getAttribute(""height"");
                    var ox = (200-w)/2
                    var oy = (200-h)/2;
                    map.setAttribute(""left"", ox + ""px"");
                    map.setAttribute(""top"", oy + ""px"");
                    */
                    for (var aid of window.avatarsInConstruct)
                    {
                        var ai = JSON.parse(CPPMod.avatarInfo(aid));
                        if (!ai.localPosition)
                            continue;
                        if (!window.avatarSvg[aid])
                        {
                            var c = document.createElementNS(""http://www.w3.org/2000/svg"", ""circle"");
                            c.setAttribute(""fill"", ""blue"");
                            c.setAttribute(""r"", 3.0);
                            var l = document.createElementNS(""http://www.w3.org/2000/svg"", ""line"");
                            l.setAttribute(""stroke"", ""red"");
                            l.setAttribute(""stroke-width"", 1.0);
                            window.avatarSvg[aid] = [c, l];
                            map.appendChild(c);
                            map.appendChild(l);
                        }
                        var c = window.avatarSvg[aid][0];
                        var l = window.avatarSvg[aid][1];
                        c.setAttribute(""cx"", ai.localPosition[0]*10.0);
                        c.setAttribute(""cy"", ai.localPosition[1]*10.0);
                        l.setAttribute(""x1"", ai.localPosition[0]*10.0);
                        l.setAttribute(""y1"", ai.localPosition[1]*10.0);
                        var q = ai.localRotation;
                        q = [q[3], q[0], q[1], q[2]];
                        var d = window.point_rotation_by_quaternion([0,1,0], q);
                        var dx = d[0]; // 2.0*(q[1]*q[2]+q[3]*q[0]);
                        var dy = d[1];// 1.0 - 2.0*(q[0]*q[0]+q[2]*q[2]);
                        l.setAttribute(""x2"", ai.localPosition[0]*10.0+dx*mmSize/20.0);
                        l.setAttribute(""y2"", ai.localPosition[1]*10.0+dy*mmSize/20.0);
                    }
                    var pi = JSON.parse(CPPMod.playerInfo());
                    var x = pi.transform[9]*10.0;
                    var y = pi.transform[10]*10.0;
                    var xmin = x - mmSize;
                    var ymin = y - mmSize;
                    var width = mmSize*2;
                    var height = mmSize*2;
                    map.setAttribute(""viewBox"",
                        xmin + "" "" + ymin + "" "" + width + "" "" + height);
            }, 200);
        }
        window.minimap.innerHTML = ""@"";
        ";
        js = js.Replace("@", Escape(svg));
        // Players ids in construct, TODO: query ConstructGrain instead
        js = "window.avatarsInConstruct = [10002, 10003, 10004, 10005, 10006];window.avatarSvg = {};" + js;
        logger.LogInformation("JAVASCRIPT: {js}", js);
        await pub.NotifyTopic(Topics.PlayerNotifications(playerId),
                    new NQutils.Messages.ModTriggerHudEventRequest(new ModTriggerHudEvent
                        {
                            eventName = "modinjectjs",
                            eventPayload = js,
                        }
                        ));
       
       // Bonus for the LOL: when a modkey is pressed bounce a laser
       // off walls a few times
       js = @"
       function bouncingLaser() {
           console.log('bouncing...');
           var rayPoints = [];
           var pi = JSON.parse(CPPMod.playerInfo());
           var wp = [pi.worldTransform[9], pi.worldTransform[10], pi.worldTransform[11]];  
           wp[2] = wp[2] + 1.5;
           rayPoints.push(wp);
           var wt = pi.worldTransform;
           var dl = [0,1,0];
           var dir = [wt[3], wt[4], wt[5]];
           var cid = 0;
           for (var i=0; i<5; i++)
           {
              console.log('dir ' + dir[0] + ' '  + dir[1] + ' ' + dir[2] + ' ');
              var start = rayPoints[rayPoints.length-1];
              var end = [start[0]+dir[0]*20, start[1]+dir[1]*20, start[2]+dir[2]*20];
              ray = JSON.parse(CPPMod.anyRay(start, end, 255));
              if (!ray.constructId)
                  break;
              cid = ray.constructId;
              var tgt = ray.impactPoint;
              rayPoints.push(tgt);
              var n = ray.impactNormal;
              var dot = dir[0]*n[0]+dir[1]*n[1]+dir[2]*n[2];
              var nplus = [n[0]*2*dot, n[1]*2*dot, n[2]*2*dot];
              var bdir  = [dir[0]-nplus[0], dir[1]-nplus[1], dir[2]-nplus[2]];
              dir = bdir;
           }
           CPPMod.sendModAction('Weapon', 66, [cid], JSON.stringify(rayPoints));
       }
       engine.on('modKey', k => bouncingLaser());
       ";
       await pub.NotifyTopic(Topics.PlayerNotifications(playerId),
                    new NQutils.Messages.ModTriggerHudEventRequest(new ModTriggerHudEvent
                        {
                            eventName = "modinjectjs",
                            eventPayload = js,
                        }
                        ));
    }
    // Receives and show a bouncinglaser request
    public async Task BouncingLaser(ulong playerId, ulong constructId, List<List<double>> coords)
    {
        for (var i=0; i<coords.Count-1; i++)
        {
            var pa = new NQ.Vec3{x=coords[i][0], y = coords[i][1], z = coords[i][2]};
            var pb = new NQ.Vec3{x=coords[i+1][0], y = coords[i+1][1], z = coords[i+1][2]};
            var ws = new WeaponShot
            {
                id = nextId++,
                originConstructId = 10000,
                weaponId = 0,
                // Picking laser because it as the shortest FX (data/resources_generated/effects.nqdef)
                weaponType = bank.GetDefinition("WeaponLaserExtraSmallAgile3").Id,
                ammoType = bank.GetDefinition("AmmoLaserExtraSmallThermicAdvancedAgile").Id,
                //weaponType = bank.GetDefinition("WeaponMissileLarge1").Id,
                //ammoType = bank.GetDefinition("AmmoMissileLargeAntimatterAdvancedAgile").Id,
                //weaponType = bank.GetDefinition("WeaponRailgunExtraSmallAgile3").Id,
                //ammoType = bank.GetDefinition("AmmoRailgunExtraSmallAntimatterAdvancedAgile").Id,
                originPositionLocal = pa,
                originPositionWorld = pa,
                targetConstructId = constructId,
                impactPositionLocal = pb,
                impactPositionWorld = pb,
            };
            // FX
            var op = new NQutils.Messages.WeaponShot(ws);
            var ev = NQutils.Serialization.Grpc.MakeEvent(op);
            var loc = new SimpleLocation
            {
                ConstructId = 0, // action.constructId,
                Position = pa, // tr.position,
            };
            await client.PublishGenericEventAsync(new EventLocation
                {
                    Event = ev,
                    Location = loc,
                    VisibilityDistance =  4000,
                });
        }
    }
    
    
    
    public async Task TriggerAction(ulong playerId, ModAction action)
    {
        /*if (!registered.ContainsKey(playerId))
        { // register any key to trigger a raycast, which will trigger a shot
            registered.Add(playerId, true);
            await pub.NotifyTopic(Topics.PlayerNotifications(playerId),
                    new NQutils.Messages.ModTriggerHudEventRequest(new ModTriggerHudEvent
                        {
                            eventName = "modinjectjs",
                            eventPayload = "engine.on(\"modKey\", function(k) {CPPMod.sendModAction(\"Weapon\", 1001, [], CPPMod.raycast());});",
                        }
                        ));
        }*/
        
        if (action.actionId == 66)
        {
            logger.LogInformation("bounce {payload}", action.payload);
            await BouncingLaser(playerId, action.constructId, JsonConvert.DeserializeObject<List<List<double>>>(action.payload));
            return;
        }
        if (action.actionId == 6)
        {
            await InjectMinimap(playerId);
        }
        if (action.actionId == 1)
        {
            var js=@"
             window.check = function(aid) {
              var ai = JSON.parse(CPPMod.avatarInfo(aid));
              var pi = JSON.parse(CPPMod.playerInfo());
              var s = [pi.worldTransform [9], pi.worldTransform[10], pi.worldTransform[11]];
              var d = ai.worldPosition;
              s[2] += 1.0;
              d[2] += 1.0;
              var r = [d[0]-s[0], d[1]-s[1], d[2]-s[2]];
              var rlen = Math.sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]);
              var start = [s[0]+r[0]*0.5/rlen, s[1]+r[1]*0.5/rlen, s[2]+r[2]*0.5/rlen];
              var end = [d[0]+r[0]/rlen, d[1]+r[1]/rlen, d[2]+r[2]/rlen];
              var ray = JSON.parse(CPPMod.anyRay(start, end, 255));
              var vis = ray.playerId == aid;
              CPPMod.sendModAction(""Weapon"", 3, [0,0,aid], vis ? ""true"" : ""false"");
            };
            ";
            building.targetPlayerId = playerId;
            await pub.NotifyTopic(Topics.PlayerNotifications(playerId),
                    new NQutils.Messages.ModTriggerHudEventRequest(new ModTriggerHudEvent
                        {
                            eventName = "modinjectjs",
                            eventPayload = js,
                        }
                        ));
            return;
        }
        if (action.actionId == 3)
        {
            foreach (var c in npcs)
            {
                if (c.playerId == action.playerId)
                {
                    await c.receiveCheck(action.payload == "true");
                    break;
                }
            }
            return;
        }
        if (action.actionId == 2)
        {
            logger.LogInformation("RC: {playerId} {raycast}", action.playerId, action.payload);
            var rc = JsonConvert.DeserializeObject<Raycast>(action.payload);
            foreach (var c in npcs)
            {
                if (c.playerId == action.playerId)
                {
                    await c.receiveRaycast(rc);
                    break;
                }
            }
            return;
        }
        if (action.actionId == 10003)
        {
            await pub.NotifyTopic(Topics.PlayerNotifications(playerId),
                    new NQutils.Messages.ModTriggerHudEventRequest(new ModTriggerHudEvent
                        {
                            eventName = "modinjectjs",
                            eventPayload = "console.log(CPPMod.playerInfo());",
                        }
                        ));
        }
        if (action.actionId == 20000)
        {
            await pub.NotifyTopic(Topics.PlayerNotifications(playerId),
                    new NQutils.Messages.ModTriggerHudEventRequest(new ModTriggerHudEvent
                        {
                            eventName = "modinjectjs",
                            eventPayload = "CPPMod.sendModAction(\"Weapon\", 20001, [], CPPMod.raycast());",
                        }));
            return;
        }
        if (action.actionId == 20001)
        {
            await AddHacker(playerId, action);
        }
        if (action.actionId == 10004)
        { // Repulsor
            var js =@"
            {
              var rcs = CPPMod.raycast();
              var rc = JSON.parse(rcs);
              var repulsorpos = rc.impactPoint;
              function dist(a, b) {
                  return Math.sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]));
              };
              window.setInterval(function() {
                  var pis = CPPMod.playerInfo();
                  //console.log(pis);
                  var pi = JSON.parse(pis);
                  //var pp = [pi.worldTransform[9], pi.worldTransform[10], pi.worldTransform[11]];
                  var pp = pi.centerOfMass;
                  var dst = dist(pp, repulsorpos);
                  console.log(dst);
                  if (dst > 20)
                    return;
                  // visibility check
                  var diff = [pp[0]-repulsorpos[0], pp[1]-repulsorpos[1], pp[2]-repulsorpos[2]];
                  var mf = 0.5 / dst;
                  var rstart = [repulsorpos[0]+diff[0]*mf, repulsorpos[1]+diff[1]*mf, repulsorpos[2]+diff[2]*mf];
                  var rend = pp;
                  var rcs = CPPMod.anyRay(rstart, rend, 255);
                  console.log(repulsorpos);
                  console.log(rstart);
                  console.log(pp);
                  console.log(rcs);
                  var rc = JSON.parse(rcs);
                  // I'm guessing self player is not in scene, so can't raycast?
                  // Or there is a bug
                  if (rc.constructId == 0) {
                      console.log(666);
                      // repulse
                      // addforce only works when jetpack is on/no contact it seems
                      CPPMod.playerAddForce(diff[0]*mf*1000, diff[1]*mf*1000, diff[2]*mf*1000);
                      var factor = mf*10/dst;
                      CPPMod.playerSetVelocity(diff[0]*factor, diff[1]*factor, diff[2]*factor);
                  }
              }, 100);
            };
            ";
            await pub.NotifyTopic(Topics.PlayerNotifications(playerId),
                    new NQutils.Messages.ModTriggerHudEventRequest(new ModTriggerHudEvent
                        {
                            eventName = "modinjectjs",
                            eventPayload = js
                        }));
        }

        // Code for weapon action
        if (debouncer.TryGetValue(playerId, out var lastShot))
        {
            if (DateTime.Now-lastShot < TimeSpan.FromMilliseconds(2000))
                return;
        }
        if (action.actionId == 1000)
        {
           await pub.NotifyTopic(Topics.PlayerNotifications(playerId),
                    new NQutils.Messages.ModTriggerHudEventRequest(new ModTriggerHudEvent
                        {
                            eventName = "modinjectjs",
                            eventPayload = "CPPMod.sendModAction(\"Weapon\", 1001, [], CPPMod.raycast())});",
                        }));
            return;
        }
        if (action.actionId == 1001)
        {
            logger.LogInformation("Got raycast {rc}", action.payload);
            var raycast = JsonConvert.DeserializeObject<Raycast>(action.payload);
            var ma = new ModAction
            {
                actionId = 2,
                playerId = raycast.playerId,
                constructId = raycast.constructId,
                elementId = raycast.elementId,
                payload = JsonConvert.SerializeObject(raycast.impactPoint),
            };
            action = ma;
            // continue
        }
        debouncer[playerId] = DateTime.Now;
        var dmg = action.actionId == 1 ? 50 : 200;
        if (action.playerId != 0)
        {
            await orleans.GetPlayerGrain(action.playerId)
                .PlayerDieOperation(new PlayerDeathInfo
                    {
                        reason = DeathReason.AnchorElementDestructionPVP,
                    });
        }
        if (action.constructId != 0)
        {
            await Shot(playerId, action);
        }
        if (false && action.elementId != 0)
        {
            await orleans.GetConstructDamageElementsGrain(action.constructId)
                .ApplyElementDamage(playerId,
                    new ElementDamageOperation
                    {
                        context = DamageContext.KINETIC,
                        elements = new List<ElementDamage>
                        {
                            new ElementDamage
                            {
                                elementId = action.elementId,
                                damage = dmg
                            }
                        },
                        damageSourcePlayerId = playerId,
                        damageSourceConstructId = action.constructId,
                    });
        }
    }
}