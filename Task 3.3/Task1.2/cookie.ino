
//note: I'm using  SEN136B5B an ultrasonic range finder from Seeedstudio 

int pins[]={3,5,2,4};// pins which the SIG of each sensor is connected
float dim[]={0.0,0.0,0.0,0.0};// contains the current reading
float prevdim[]={0.0,0.0,0.0,0.0};// contains the previous reading
// The indices of the movement
const int u=0;
const int d=1;
const int r=2;
const int l=3;
int found=-1;
// Function that reads the data from the sensor
long readUltrasonicDistance(int triggerPin, int echoPin)
{
  pinMode(triggerPin, OUTPUT);  
  // Clear the trigger
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  // Sets the trigger pin to HIGH state for 10 microseconds
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT);
  // Reads the echo pin, and returns the sound wave travel time in microseconds
  return pulseIn(echoPin, HIGH);
}

// Function that checks if the gun is found
bool gunFound ()
{
  if(6-dim[u]-dim[d]>0.1 || 5-dim[l]-dim[r]>0.1)
    return true;
  return false;
}

// Function checks if the cookie moved taking in consideration 0.01 meter noise  
bool check()
{  
  for(int i=0;i<4;i++){
    if(prevdim[i]-dim[i] > 0.01){
      return false;
    }
  }  
  return true;
}

// Function checks the movment vertically
bool vertical()
{
  if(prevdim[u]-dim[u] > 0.1 && prevdim[u]-dim[u] > 0.1 && 6-dim[0]-dim[1]>0.2)
    return true;
  return false;
}

// Function checks the movment horizontally
bool horizontal()
{
  if(prevdim[r]-dim[r] > 0.1 && prevdim[l]-dim[l] > 0.1 && 6-dim[r]-dim[l]>0.2)
    return true;
  return false;
}

// function checks if the current reading is equal to the previous one taking in consideration 0.1 meter noise
// i parameter is the index we want to check on "up or down or right or left" 
bool equal (int i)
{
  if (prevdim[i]-dim[i] < 0.1)
    return true;
  return false;		
}

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  
  // taking the previous data
  for(int i=0;i<4;i++){
  	prevdim[i]=dim[i];
  }
  // if the gun is not fount make found -1 (for checking on it later)
  if(!gunFound())
  {
    found=-1;
  }
  for(int i=0;i<4;i++)
  {
  	// measure the ping time in m
   	dim[i] = 0.01723 * readUltrasonicDistance(pins[i], pins[i])/100;
  
  	delay(100); // Wait for 100 millisecond(s)
  }

   // Checking if there is movement 
   // Cheking if the movement is vertical and there is no obstacle in the left
   // Cheking if the movement is horizontal and there is no obstacle down
   // checking if no obstacle is in the range of the sensors
   // printing the dimension from the left and from down
  if((!check() && (vertical() && equal(l)  || horizontal() && equal(d))) && found==-1)
  {
      Serial.print("dimensions are ( ");
      Serial.print(dim[l]);
      Serial.print(",");
      Serial.print(dim[d]);
      Serial.println(" )");
   }
   // checking to see if there was a movement 
   else if (!check())
    {
      // seeing if the obstacle faced is at the left
      // changing by taking the dimension from the right
     if(vertical() || found ==l)
     {
          Serial.print("dimensions are ( ");
          Serial.print(5-dim[r]);
          Serial.print(",");
          Serial.print(dim[d]);
          Serial.println(" )");
          found = l;
     }
	 
      // seeing if the obstacle faced is down 
      // changing by taking the dimension from up
     else if(horizontal()  || found == d)
     {
          Serial.print("dimensions are ( ");
          Serial.print(dim[l]);
          Serial.print(",");
          Serial.print(6-dim[u]);
          Serial.println(" )");
          found=d;
     }
     
    }
  //just a delay for testing
  delay(100);
}
