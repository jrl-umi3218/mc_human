{
  description = "mc-human: human robot module for mc-rtc";

  inputs = {
    mc-rtc-nix.url = "github:mc-rtc/nixpkgs";
    flake-parts.follows = "mc-rtc-nix/flake-parts";
    systems.follows = "mc-rtc-nix/systems";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.mc-rtc-nix.flakeModule
          {
            mc-rtc-superbuild =
              { pkgs, ... }:
              {
                enable = true;
                configurations = {
                  mc-human = {
                    extends = [ "minimal" ];
                    runtime = {
                      apps = [
                        pkgs.mc-rtc-magnum
                      ];
                    };
                    devel = {
                      robots = [ pkgs.mc-human ];
                    };
                  };
                };
              };

            flakoboros = {
              # Override all dependencies
              # They are locked in flake.lock to the latest commit available at the time
              # To update to all inputs' latest commit, use
              # nix flake update
              overrideAttrs.mc-human = {
                src = lib.cleanSource ./.;
              };
            };
          }
        ];
      }
    );
}
