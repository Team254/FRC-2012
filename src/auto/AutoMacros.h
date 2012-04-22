#ifndef AUTO_MACROS_H
#define AUTO_MACROS_H


// Drive up bridge and get balls
#define BALLS_FROM_BRIDGE_COMMAND(runIntake)  AUTO_SEQUENTIAL( \
                                      new SetIntakePositionCommand(intake_, Intake::INTAKE_DOWN), \
                                      new OldDriveCommand(drivebase_, -10, 0.0, false, .5, 1.0), \
                                      new DelayCommand(.25), \
                                      AUTO_CONCURRENT( \
                                      new OldDriveCommand(drivebase_, 25, 0.0, false, .5, 1.0), \
                                      AUTO_SEQUENTIAL( \
                                        new DelayCommand(.25), \
                                        new BridgeBallsCommand(intake_, shooter_, runIntake, 1.9))))

#define AUTO_SHOOT_COMMAND(speed, offset, useSkew)  AUTO_CONCURRENT( \
                new AutoAlignCommand(drivebase_, autoAlignDriver_, target_, offset, 5, useSkew), \
                AUTO_SEQUENTIAL( \
                  new DelayCommand(.2), \
                  new ShootFieldCommand(shooter_, intake_, true, speed, 10, 10.0)))
#endif
