
int getMicrosecondsForward(float percentage)
{
	if(percentage > 0 && percentage <= 100)
	{
		return 1525 + ((percentage/100)*375);
	}
	else return 1500; //any unexpected values will stop the motor
}

int getMicrosecondsBackward(float percentage)
{
	if(percentage > 0 && percentage <= 100)
	{
		return 1475 - ((percentage/100)*375);
	}
	else return 1500; //any unexpected values will stop the motor
}
