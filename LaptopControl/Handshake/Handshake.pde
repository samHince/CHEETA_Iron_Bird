import processing.serial.*;
import java.io.FileWriter;
import java.io.BufferedWriter;
import java.util.stream.Collectors;

ArrayList<Integer> times = new ArrayList<Integer>();
ArrayList<Integer> throttles = new ArrayList<Integer>();


int bgcolor;           // Background color

int fgcolor;           // Fill color

Serial myPort;                       // The serial port

ArrayList<Integer> Data_Recieved = new ArrayList<Integer>();    // Where we'll put what we receive

int serialCount = 0;                 // A count of how many bytes we receive

boolean firstContact = false;        // Whether we've heard from the microcontroller
Integer time;
long secondTime;
long firstTime;
int current = 0;
String data_in = "";
boolean gather = false;

void setup() {

  size(500, 500);  // Stage size

  noStroke();      // No border on the next thing drawn

  // Change the 2 to 0 for MAC assuming your using a port extension thing
  // If that doesn't work un-comment following line and try another index
  
  // print(Serial.list());
  
  String portName = Serial.list()[2];

  myPort = new Serial(this, portName, 9600);
  
  readCSV();
  println("Read CSV");
}

void draw() {

  background(bgcolor);

  fill(fgcolor);
  if (Data_Recieved.size() > 1) {
    //String s = Data_Recieved.stream().map(Object::toString)
    //                      .collect(Collectors.joining("\t"));

    //text(s, 40, 40, 280, 320);
  }
}

void serialEvent(Serial myPort) {

  // read a byte from the serial port:

  char inByte = (char) myPort.read();
 

  // if this is the first byte received, and it's an A, clear the serial

  // buffer and note that you've had first contact from the microcontroller.

  // Otherwise, add the incoming byte to the array:

  if (firstContact == false) {
    println("Attempting Contact");
    if (inByte == 'a') {

      myPort.clear();          // clear the serial port buffer
      println("Connected");
      firstTime = System.currentTimeMillis()/1000;



      firstContact = true;     // you've had first contact from the microcontroller
      myPort.write('a');
      
      myPort.write("t0");
      myPort.write('a');
      // technically this can cause data loss but it would need to run for 24 days straight so im not going to bother
      
    } else {
      myPort.clear();
    }
  } else {
    // Add the latest byte from the serial port to array:
    
    secondTime = (System.currentTimeMillis()/1000);
    time = (int) (long) (secondTime - firstTime);
    //print("Time at Byte: ");
    //println(time);
    
    // move to next throttle so there isn't a lag
    if (time > times.get(current)) {
      myPort.write("t" + str(throttles.get(current)));
      current++;
      delay(300);
      
      myPort.write('a');
      gather = true;
    }
    
    // Sends something back when serial events stop
    if ((time <= times.get(current)) && inByte == 'a' && !gather) {
      myPort.clear();
      return;
    }

    if (inByte == 'a') {
      if (Data_Recieved.size() < 4) {
        //myPort.write('a');
        return;
      }

      writeCSV();
      
      if (serialCount > 100) {
        redraw();
        serialCount = 0;
      }

      // Setting up for new interval
      secondTime = (System.currentTimeMillis()/1000);
      time = (int) (long) (secondTime - firstTime);
      
      Data_Recieved.clear();
      Data_Recieved.add(time);
      
      
      serialCount++;
      data_in = "";
      gather = false;
      
    } else if (inByte != 'a') {
      if (inByte != ' ') {
        data_in += inByte;
      } else {
        int newC = Integer.valueOf(data_in);
        data_in = "";
        Data_Recieved.add(newC);

      }
    }
  }
}

void writeCSV() {
  FileWriter output = null;
  try {
  //println("Write");
  output = new FileWriter("C:/Users/marym/OneDrive/Desktop/Handshake/Data_0508_1613.txt", true);
  for (Integer i : Data_Recieved) {
      print("Writing: ");
      println(i);
      output.write(str(i));
      output.write('\t');
  }
  output.write('\n');
  println("Written");
  } catch (IOException e) {
    println("It Broke");
    e.printStackTrace();
    throw new RuntimeException(e);
  }
  try {
    output.close();
    //println("Written!");
  } catch (IOException e) {
    println("Error Closing File");
  }
}

void readCSV() {
  BufferedReader reader = createReader("example.txt");
  String line = null;
  try {
    while ((line = reader.readLine()) != null) {
      String[] pieces = split(line, ' ');
      times.add(int(pieces[0]));
      throttles.add(int(pieces[1]));
    }
    reader.close();
  } catch (IOException e) {
    e.printStackTrace();
  }
  println(times);
  println(throttles);
}
