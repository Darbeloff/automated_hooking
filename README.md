# Automated Hooking
Code and content for autonomously hooking horizontal pegs with a gantry crane

## Setup

TODO

## Parts and Manufacture

[Parts List](Parts_List.md)

Manufacture is TODO

## Attachment Algorithm Overview

**Set Initial Condition**
> - Use camera to detect peg location with mounted April Tag
> - Compute bounding box of acceptable initial conditions to the right of the peg
> - Use crane to move hook to the center of this bounding box (this can be imprecise)

**Alignment**
> - WHILE True
> > - Move crane left until hook-mounted accelerometer detects an impact
> > - Use combination of linear and rotational acceleration to determine position of impact
> > - IF impact was detected on below the lip of the hook
> > > - Lower Z position of the hook
> > > - Back off and try again
> > - IF impact was detected on the back side of the hook
> > > - Back off and try again
> > - IF neither
> > > - break

**Hooking**
> - Move crane left to continue pulling the hook until ammeters in the crane detect the line is taut
> - Slack the line to release tension

 **Verification**
> - Move crane to be vertically above the peg
> - Winch in line until ammeters in the crane detect the line is taut
> - Use information from the hook-mounted accelerometer to determine if impact was on the inner surface of the hook
> - IF so, DONE!
> - ELSE
> > - Go to Detachment process, and begin again from **set initial condition**

**Detatchment**
> - TODO