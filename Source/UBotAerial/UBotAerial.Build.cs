using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using EpicGames.Core;
using UnrealBuildTool;

public class UBotAerial : ModuleRules
{
    public UBotAerial(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new[]
        {
            "Core",
            "CoreUObject",
            "Engine",
            "EnhancedInput",
            "InputCore",
            "UBotCore"
        });

        bool bWithEnvironment = IsEnvironmentEnabled(Target);
        PrivateDefinitions.Add("WITH_UBOT_ENVIRONMENT=" + (bWithEnvironment ? "1" : "0"));
        if (bWithEnvironment)
        {
            PrivateDependencyModuleNames.Add("UBotEnvironment");
        }
    }

    private bool IsEnvironmentEnabled(ReadOnlyTargetRules Target)
    {
        ProjectDescriptor Project = null;
        if (Target.ProjectFile != null)
        {
            ExternalDependencies.Add(Target.ProjectFile.FullName);
            Project = ProjectDescriptor.FromFile(Target.ProjectFile);
        }

        var Discovered = Plugins.ReadAvailablePlugins(new DirectoryReference(EngineDirectory),
            Target.ProjectFile?.Directory, Project?.AdditionalPluginDirectories);
        // A foreign plugin, including BuildPlugin's input, need not be discoverable from the project.
        var AerialFile = new FileReference(Path.Combine(PluginDirectory, "UBotAerial.uplugin"));
        if (!Discovered.Any(Info => Info.File == AerialFile))
        {
            Discovered.Add(new PluginInfo(AerialFile, PluginType.External));
        }
        var Available = Plugins.ToFilteredDictionary(Discovered);
        foreach (PluginInfo Info in Discovered)
        {
            // Disabled/default plugins can change the enablement closure without changing C++.
            ExternalDependencies.Add(Info.File.FullName);
            ExternalDependencies.AddRange(Info.ChildFiles.Select(File => File.FullName));
        }
        if (!Available.ContainsKey("UBotEnvironment"))
        {
            return false;
        }

        // Match UBT's root-reference order: target enable, disable, optional, project, defaults.
        var Roots = new Dictionary<string, PluginReferenceDescriptor>(StringComparer.OrdinalIgnoreCase);
        foreach (string Name in Target.EnablePlugins)
        {
            Roots.TryAdd(Name, new PluginReferenceDescriptor(Name, null, true));
        }
        foreach (string Name in Target.DisablePlugins)
        {
            Roots.TryAdd(Name, new PluginReferenceDescriptor(Name, null, false));
        }
        foreach (string Name in Target.OptionalPlugins)
        {
            Roots.TryAdd(Name, new PluginReferenceDescriptor(Name, null, true) { bOptional = true });
        }
        if (Project?.Plugins != null)
        {
            foreach (PluginReferenceDescriptor Reference in Project.Plugins)
            {
                Roots.TryAdd(Reference.Name, Reference);
            }
        }

        bool IsReferenceEnabled(PluginReferenceDescriptor Reference) =>
            Reference.IsEnabledForPlatform(Target.Platform)
            && Reference.IsEnabledForTargetConfiguration(Target.Configuration)
            && Reference.IsEnabledForTarget(Target.Type)
            && (Target.bIncludePluginsForTargetPlatforms || Reference.IsSupportedTargetPlatform(Target.Platform));

        // Aerial's optional integration must respect an explicit Environment disable, even if
        // another plugin has a required reference that makes UBT build Environment regardless.
        if (Roots.TryGetValue("UBotEnvironment", out var ExplicitEnvironment)
            && !IsReferenceEnabled(ExplicitEnvironment))
        {
            return false;
        }

        // Module rules can also run for foreign/plugin-only builds with no project reference.
        Roots.TryAdd("UBotAerial", new PluginReferenceDescriptor("UBotAerial", null, true));
        if (Target.bCompileAgainstEngine || Target.bCompileWithPluginSupport)
        {
            bool bAllowEngineDefaults = Target.bAllowEnginePluginsEnabledByDefault
                && !(Project?.DisableEnginePluginsByDefault ?? false);
            foreach (PluginSet Versions in Available.Values)
            {
                PluginInfo Info = Versions.ChoiceVersion;
                if (Info != null && Info.IsEnabledByDefault(bAllowEngineDefaults))
                {
                    Roots.TryAdd(Info.Name, new PluginReferenceDescriptor(Info.Name, null, true)
                    {
                        bOptional = true,
                        SupportedTargetPlatforms = Info.Descriptor.GetSupportedTargetPlatformNames(),
                        bHasExplicitPlatforms = Info.Descriptor.bHasExplicitPlatforms,
                        PlatformAllowList = Info.Descriptor.bHasExplicitPlatforms
                            ? Info.Descriptor.GetSupportedTargetPlatformNames() : null
                    });
                }
            }
        }

        // UE 5.5 exposes discovery and descriptor filters, but no resolved target plugin set.
        // Follow only enablement references; BuildPlugins does not enable runtime integration.
        var Visited = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        bool EnablesEnvironment(PluginReferenceDescriptor Reference)
        {
            if (!IsReferenceEnabled(Reference) || !Available.TryGetValue(Reference.Name, out var Versions))
            {
                return false;
            }
            PluginInfo Info = Reference.RequestedVersion.HasValue
                ? Versions.KnownVersions.FirstOrDefault(Candidate => Candidate.Descriptor.Version == Reference.RequestedVersion.Value)
                : Versions.ChoiceVersion;
            if (Info == null
                || (!Target.bIncludePluginsForTargetPlatforms && !Info.Descriptor.SupportsTargetPlatform(Target.Platform))
                || (Target.Type == TargetType.Program && !(Info.Descriptor.SupportedPrograms?.Contains(Target.Name) ?? false))
                || !Visited.Add(Info.Name))
            {
                return false;
            }
            if (Info.Name.Equals("UBotEnvironment", StringComparison.OrdinalIgnoreCase))
            {
                return Info.Descriptor.Modules?.Any(Module => Module.Name == "UBotEnvironment"
                    && Module.IsCompiledInConfiguration(Target.Platform, Target.Configuration, Target.Name,
                        Target.Type, Target.bBuildDeveloperTools, Target.bBuildRequiresCookedData)) ?? false;
            }
            return Info.Descriptor.Plugins?.Any(Dependency =>
                !Target.ShouldIgnorePluginDependency(Info, Dependency) && EnablesEnvironment(Dependency)) ?? false;
        }
        return Roots.Values.Any(EnablesEnvironment);
    }
}
