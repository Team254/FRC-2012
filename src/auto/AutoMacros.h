#ifndef AUTO_MACROS_H
#define AUTO_MACROS_H


// Drive up bridge and get balls
#define BALLS_FROM_BRIDGE_COMMAND()  AUTO_CONCURRENT( \
                                       AUTO_SEQUENTIAL( \
                                         new OldDriveCommand(drivebase_, -10, 0.0, false, .5, 1.0), \
                                         new DelayCommand(.75), \
                                         new OldDriveCommand(drivebase_, 20, 0.0, false, .5, 1.0)), \
                                       new BridgeBallsCommand(intake_, shooter_, true, 3.2))


#endif