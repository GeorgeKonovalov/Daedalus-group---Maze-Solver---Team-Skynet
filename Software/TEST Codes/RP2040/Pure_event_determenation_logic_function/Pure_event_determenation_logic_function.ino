
// After IR set timed recheck the event and act only if those are identical (otherwise wait again and compare with the previous one) + add number of identical events in a row for considering event to be valid
// Now events can be triggered only within the corridor event (when current state is the corridor)
char Event_Determination(bool IR_front_right; bool IR_front_left; bool IR_rear_right; bool IR_rear_left; float Ultrasonic_front; float Ultrasonic_left; float Ultrasonic_right; float front_Dead_End_threashold; float front_Turn_Threashold; float front_Wall_threashold)
{
  if (Ultrasonic_front <= front_Wall_Threashold)
  {
    return 's'; // Just a full stop of the machine (something went wrong cause other events supposed to be triggered before)
    break;
  }
  else if (Ultrasonic_front <= front_Dead_End_Threashold)
  {
    if (IR_front_right == HIGH && IR_front_left == HIGH) //both front IRs are triggered
    {
      return 'd'; // returns dead end - if front IR are triggered and the distance to the wall is less than this threashold (walls are aside and no passage in front)
      break;
    }
  }
}





void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
