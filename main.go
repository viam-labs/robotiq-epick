// Package main is the entry point for the Robotiq EPick vacuum gripper module.
package main

import (
	"context"

	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/module"
	"go.viam.com/utils"

	"github.com/viam-labs/robotiq-epick/epick"
)

func main() {
	utils.ContextualMain(mainWithArgs, module.NewLoggerFromArgs("robotiq-epick"))
}

func mainWithArgs(ctx context.Context, args []string, logger logging.Logger) error {
	mod, err := module.NewModuleFromArgs(ctx)
	if err != nil {
		return err
	}

	if err = mod.AddModelFromRegistry(ctx, gripper.API, epick.Model); err != nil {
		return err
	}

	err = mod.Start(ctx)
	defer mod.Close(ctx)
	if err != nil {
		return err
	}

	<-ctx.Done()
	return nil
}
