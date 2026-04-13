
// After IR set timed recheck the event and act only if those are identical (otherwise wait again and compare with the previous one) + add number of identical events in a row for considering event to be valid
// Now events can be triggered only within the corridor event (when current state is the corridor)
char Event_Determination(bool IR_front_right; bool IR_front_left; bool IR_rear_right; bool IR_rear_left; float Ultrasonic_front; float Ultrasonic_left; float Ultrasonic_right; float front_Dead_End_threashold; float front_Turn_Threashold; float front_Wall_Threashold, float side_Wall_Threashold)
{
  if (Ultrasonic_front <= front_Wall_Threashold)
  {
    return 's'; // Just a full stop of the machine (something went wrong cause other events supposed to be triggered before)
    break;
  }
  else if (Ultrasonic_front <= front_Dead_End_Threashold)
  {
    if (IR_front_right == HIGH) //right side is blocked
    {
      if (IR_front_left == HIGH) // left side is also blocked
      {
        return 'd'; // returns dead end - if front IR are triggered and the distance to the wall is less than this threashold (walls are aside and no passage in front)
        break;
      }
      else // left side isn't blocked
      {
        return 'l'; // returns left turning (front - closed, right -closed, left - open)
        break;
      }
    }
    else // right side is free
    {
      if (IR_front_left == HIGH)
      {
        return 'r'; // returns right-turn (front and left are closed, right - open)
        break;
      }
      else // both IR are not triggered
      {
        return 't'; // return T-junction (front - closed, sides - open)
        break;
      }
    }
  }
  else if (Ultrasonic_front <= front_Turn_Threashold)
  {
    if (IR_front_right == HIGH) //right side is blocked
    {
      if (IR_front_left == HIGH) // left side is also blocked
      {
        return 'c'; // returns corridor - if front IR are triggered and the distance to the wall is less than this threashold (walls are aside and no passage in front)
        break;
      }
      else // left side isn't blocked
      {
        return 'l'; // returns left turning (front - closed, right -closed, left - open)
        break;
      }
    }
    else // right side is free
    {
      if (IR_front_left == HIGH)
      {
        return 'r'; // returns right-turn (front and left are closed, right - open)
        break;
      }
      else // both IR are not triggered
      {
        return 't'; // return T-junction (front - closed, sides - open)
        break;
      }
    }

  }
  else // distance is higher then the Turn Threashold
  {
    if (IR_front_right == HIGH)
    {
      if (IR_front_left == HIGH)
      {
        return 'c'; // returns corridor - if front IR are triggered and the distance to the wall is less than this threashold (walls are aside and no passage in front)
        break;
      }
      else
      {
        return 't' // T-junction straight(no matter what turning is on)
        break;
      }
    }
    else // front IR == LOW
    {
      if (IR_front_left == HIGH)
      {
        return 't' // T-junction straight(no matter what turning is on)
        break;
      }
      else // IR_front_left == LOW
      {
        if (IR_rear_left == LOW && IR_rear_right == LOW && Ultrasonic_left >= side_Wall_Threashold && Ultrasonic_right >= side_Wall_Threashold)
        {
          return 'f' // finish
          break;
        }
      }
    }
  }
  return 'c'; // in all other cases corridor
  break;
}

void PID_selection(float Ultrasonic_left; float Ultrasonic_right; float side_Wall_Threashold)
{
  if (Ultrasonic_left <= side_Wall_Threashold)
  {
    if (Ultrasonic_right <=side_Wall_Threashold)
    {
      //use 2-side PID
    }
    else
    {
      //use 1-side PID left-side as an desired
    }
  }
  else
  {
    if (Ultrasonic_right <=side_Wall_Threashold)
    {
      //use 1-side PID - right side
    }
    else
    {
      //use no PID (no reliable sources of location)
    }
  }
}




void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
