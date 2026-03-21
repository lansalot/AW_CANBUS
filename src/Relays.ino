#ifndef Relays_H
#define Relays_H

// Hitch Control------------------------------------------------------------
void SetRelaysFendt(void)
{
  if (goDown)
    liftGo(); // Lift Go button if pressed - CAN Page
  if (endDown)
    liftEnd(); // Lift End button if pressed - CAN Page

  // If Invert Relays is selected in hitch settings, Section 1 is used as trigger.
  if (aogConfig.isRelayActiveHigh == 1)
  {
    bitState = (bitRead(relay, 0));
  }
  // If not selected hitch command is used on headland used as Trigger
  else
  {
    if (hydLift == 1)
      bitState = 1;
    if (hydLift == 2)
      bitState = 0;
  }
  // Only if tool lift is enabled AgOpen will press headland buttions via CAN
  if (aogConfig.enableToolLift == 1)
  {
    if (bitState && !bitStateOld)
      pressGo(); // Press Go button - CAN Page
    if (!bitState && bitStateOld)
      pressEnd(); // Press End button - CAN Page
  }

  bitStateOld = bitState;
}

void SetRelaysClaas(void)
{
  // If Invert Relays is selected in hitch settings, Section 1 is used as trigger.
  if (aogConfig.isRelayActiveHigh == 1)
  {
    bitState = (bitRead(relay, 0));
  }
  // If not selected hitch command is used on headland used as Trigger
  else
  {
    if (hydLift == 1)
      bitState = 1;
    if (hydLift == 2)
      bitState = 0;
  }
  // Only if tool lift is enabled AgOpen will press headland buttions via CAN
  if (aogConfig.enableToolLift == 1)
  {
    if (bitState && !bitStateOld)
      pressCSM1(); // Press Go button - CAN Page
    if (!bitState && bitStateOld)
      pressCSM2(); // Press End button - CAN Page
  }

  bitStateOld = bitState;
}
#endif