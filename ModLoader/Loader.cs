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


public class MyDuMod: IMod
{
    private IServiceProvider isp;
    private ILogger logger;
    public string GetName()
    {
        return "Loader";
    }
    public Task Initialize(IServiceProvider isp)
    {
        this.isp = isp;
        this.logger = isp.GetRequiredService<ILogger<MyDuMod>>();
        return Task.CompletedTask;
    }
    public async Task<ModInfo> GetModInfoFor(ulong playerId, bool admin)
    {
        await SendTo(playerId);
        return new ModInfo
        {
            name = GetName(),
            actions = new List<ModActionDefinition>
            {
                new ModActionDefinition
                {
                    id = 1000,
                    label = "Sound 1",
                    context = ModActionContext.Global,
                },
                new ModActionDefinition
                {
                    id = 1001,
                    label = "Sound 2",
                    context = ModActionContext.Global,
                },
                new ModActionDefinition
                {
                    id = 1002,
                    label = "Sound 3",
                    context = ModActionContext.Global,
                },
                new ModActionDefinition
                {
                    id = 1003,
                    label = "Sound 4",
                    context = ModActionContext.Global,
                },
                new ModActionDefinition
                {
                    id = 1,
                    label = "Sound Here",
                    context = ModActionContext.Element,
                },
            }
        };
    }
    public async Task SendTo(ulong playerId)
    {
        var pub = isp.GetRequiredService<IPub>();
        string codeBase = Assembly.GetExecutingAssembly().Location;
        string dir = Path.GetDirectoryName(codeBase) + "/client";
        var defFiles = Directory.GetFiles(dir, "*.nqdef");
        var processed = new List<string>();
        var elems = new List<string>();
        foreach (var defFile in defFiles)
        {
            processed.Add(defFile);
            var content = File.ReadAllBytes(defFile);
            var s = new VoxelEdit
            {
                flags = 1,
                operation = content
            };
            // Not generic enough, use .elr files instead...elems.Add(Path.GetFileName(defFile).Split('.')[0]);
            logger.LogInformation("sending nqdef...{name}", defFile);
            await pub.NotifyTopic(Topics.PlayerNotifications(playerId),
                new NQutils.Messages.VoxelCsgApplied(s));
        }
        await Task.Delay(1000);
        var bFiles = Directory.GetFiles(dir, "*.bnk");
        foreach (var bnk in bFiles)
        {
            processed.Add(bnk);
            var content = File.ReadAllBytes(bnk);
            var s = new VoxelEdit
            {
                flags = 4,
                operation = content
            };
            logger.LogInformation("sending BNK...{name}", bnk);
            await pub.NotifyTopic(Topics.PlayerNotifications(playerId),
                new NQutils.Messages.VoxelCsgApplied(s));
        }
        var cFiles = Directory.GetFiles(dir, "*.csv");
        foreach (var csv in cFiles)
        {
            processed.Add(csv);
            var content = File.ReadAllBytes(csv);
            var s = new VoxelEdit
            {
                flags = 5,
                operation = content,
                hashContext = "resources_generated/" + Path.GetFileName(csv),
            };
            logger.LogInformation("sending localization CSV...{name}", csv);
            await pub.NotifyTopic(Topics.PlayerNotifications(playerId),
                new NQutils.Messages.VoxelCsgApplied(s));
        }
        var elrFiles = Directory.GetFiles(dir, "*.elr");
        foreach (var elf in elrFiles)
        {
            processed.Add(elf);
            var content = File.ReadAllText(elf);
            foreach (var el in content.Split("\n"))
            {
                if (el == "")
                    continue;
                elems.Add(el);
            }
        }
        
        var allFiles = Directory.GetFiles(dir);
        foreach (var fn in allFiles)
        {
            if (processed.Contains(fn))
                continue;
            var content = File.ReadAllBytes(fn);
            var s = new VoxelEdit
            {
                flags = 2,
                operation = content,
                hashContext = "resources_generated/" + Path.GetFileName(fn),
            };
            logger.LogInformation("sending misc...{name}", fn);
            await pub.NotifyTopic(Topics.PlayerNotifications(playerId),
                new NQutils.Messages.VoxelCsgApplied(s));
        }
        await Task.Delay(1000);
       
        foreach (var el in elems)
        {
            var s = new VoxelEdit
            {
                flags = 3,
                hashContext = el,
            };
            logger.LogInformation("sending elt load...{name}", el);
            await pub.NotifyTopic(Topics.PlayerNotifications(playerId),
                new NQutils.Messages.VoxelCsgApplied(s));
        }
    }
    public async Task TriggerAction(ulong playerId, ModAction action)
    {
        if (action.actionId == 1)
        {
            await Task.Delay(1500);
            await isp.GetRequiredService<IPub>().NotifyTopic(Topics.PlayerNotifications(playerId),
            new NQutils.Messages.ModTriggerHudEventRequest(new ModTriggerHudEvent
                {
                    eventName = "modinjectjs",
                    eventPayload = $"CPPMod.soundPostEvent({action.constructId}, {action.elementId}, 607644327);",
                }));
            return;
        }
        int id = (int)action.actionId - 1000;
        List<ulong> ids = new List<ulong>{607644327, 1840582873, 607644324, 1840582874};
        await isp.GetRequiredService<IPub>().NotifyTopic(Topics.PlayerNotifications(playerId),
            new NQutils.Messages.ModTriggerHudEventRequest(new ModTriggerHudEvent
                {
                    eventName = "modinjectjs",
                    eventPayload = $"soundBinding.postSoundEvent({ids[id]});",
                }));
        
    }
}