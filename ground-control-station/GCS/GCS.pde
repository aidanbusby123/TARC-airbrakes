import processing.serial.*;
import java.awt.datatransfer.*;
import java.awt.Toolkit;
import processing.opengl.*;
import saito.objloader.*;
import g4p_controls.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.Arrays;

float roll  = 0.0F;
float pitch = 0.0F;
float yaw   = 0.0F;
float temp  = 0.0F;
float alt   = 0.0F;
float time = 0.0F;

float qw = 0.0F;
float qx = 0.0F;
float qy = 0.0F;
float qz = 0.0F;

float ax = 0.0F;
float ay = 0.0F;
float az = 0.0F;

float ax_local = 0.0F;
float ay_local = 0.0F;
float az_local = 0.0F;

float vx = 0.0F;
float vy = 0.0F;
float vz = 0.0F;

float vx_local = 0.0F;
float vy_local = 0.0F;
float vz_local = 0.0F;


int MSG_START = 0xFa;
int MSG_END = 0xFb;

int msg_size = 0;
int msg_type = 0x8a;

OBJModel model;

int accelScale = 100;

// Serial port state.
Serial       port;
String       buffer = "";
final String serialConfigFile = "serialconfig.txt";
boolean      printSerial = false;




// UI controls.
GPanel    configPanel;
GDropList serialList;
GLabel    serialLabel;
GCheckbox printSerialCheckbox;

GPanel dataPanel;
GLabel time_Label;
GLabel accel_x_Label, accel_y_Label, accel_z_Label, gyro_x_Label, gyro_y_Label, gyro_z_Label;
GLabel accel_x_local_Label, accel_y_local_Label, accel_z_local_Label;
GLabel vel_x_Label, vel_y_Label, vel_z_Label, vel_x_local_Label, vel_y_local_Label, vel_z_local_Label;
GLabel alt_Label;

void setup()
{
  size(1920, 1000, OPENGL);
 /// fullScreen();
  frameRate(30);
  model = new OBJModel(this);
  model.load("rocket.obj");
  model.scale(0.5);
  
      // do something with each field

  
  // Serial port setup.
  // Grab list of serial ports and choose one that was persisted earlier or default to the first port.

  // Build serial config UI.
  configPanel = new GPanel(this, 10, 10, width-20, 90, "Configuration (click to hide/show)");
  serialLabel = new GLabel(this,  0, 20, 80, 25, "Serial port:");
  configPanel.addControl(serialLabel);
  serialList = new GDropList(this, 90, 20, 200, 200, 6);

  configPanel.addControl(serialList);
  printSerialCheckbox = new GCheckbox(this, 5, 50, 200, 20, "Print serial data");
  printSerialCheckbox.setSelected(printSerial);
  configPanel.addControl(printSerialCheckbox);
  
  dataPanel = new GPanel(this, 10, 110, 380, 200, "Telemetry");
  dataPanel.setCollapsed(false);
  
  time_Label = new GLabel(this, 10, 20, 200, 20, "Time: 0.0");
  alt_Label = new GLabel(this, 10, 50, 200, 20, "Altitude: 0.0");
  
  accel_x_Label = new GLabel(this, 10, 80, 200, 20, "AX: 0.0");
  accel_y_Label = new GLabel(this, 10, 110, 200, 20, "AY: 0.0");
  accel_z_Label = new GLabel(this, 10, 140, 200, 20, "AZ: 0.0");
  
  accel_x_local_Label = new GLabel(this, 10, 170, 200, 20, "AX_Local: 0.0");
  accel_y_local_Label = new GLabel(this, 10, 200, 200, 20, "AY_Local: 0.0");
  accel_z_local_Label = new GLabel(this, 10, 230, 200, 20, "AZ_Local: 0.0");
  
  dataPanel.addControl(accel_x_Label);
  dataPanel.addControl(accel_y_Label);
  dataPanel.addControl(accel_z_Label);
  dataPanel.addControl(time_Label);
  
    int selectedPort = 0;
  String[] availablePorts = Serial.list();
  while (availablePorts == null) {
    println("ERROR: No serial ports available!");
    continue;
  }
  String[] serialConfig = loadStrings(serialConfigFile);
  if (serialConfig != null && serialConfig.length > 0) {
    String savedPort = serialConfig[0];
    // Check if saved port is in available ports.
    for (int i = 0; i < availablePorts.length; ++i) {
      if (availablePorts[i].equals(savedPort)) {
        selectedPort = i;
      } 
    }
  }
    serialList.setItems(availablePorts, selectedPort);
  
  // Set serial port.
 setSerialPort(serialList.getSelectedText());
}
 
void draw()
{
  
  /*while (port.available() > 0){
    int incoming_byte = port.read();
    //println(Integer.toHexString(incoming_byte));
    println(incoming_byte);
 }*/
  
  time_Label.setText("time: " + nf(time, 1, 2));
  accel_x_Label.setText("AX: " + nf(ax, 1, 2));
  accel_y_Label.setText("AY: " + nf(ay, 1, 2));
  accel_z_Label.setText("AZ: " + nf(az, 1, 2));
  background(0,0, 0);

  // Set a new co-ordinate space
  pushMatrix();


  // Simple 3 point lighting for dramatic effect.
  // Slightly red light in upper right, slightly blue light in upper left, and white light from behind.
  pointLight(255, 200, 200,  400, 400,  500);
  pointLight(200, 200, 255, -400, 400,  500);
  pointLight(255, 255, 255,    0,   0, -500);
  
  // Displace objects from 0,0
  translate(width/2, height/2, 0);
  
  
   stroke(255, 0, 0);
   strokeWeight(5);
   line(0, 0, 0, ax*accelScale, 0, 0);
   
   stroke(0, 255, 0);
   strokeWeight(5);
   line(0, 0, 0, 0, az*accelScale, 0);
   
   stroke(0, 0, 255);
   strokeWeight(5);
   line(0, 0, 0, 0, 0, ay*accelScale);

  //println(qx);
  // Rotate shapes around the X/Y/Z axis (values in radians, 0..Pi*2)
  applyMatrix(
  1.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 1.0F, 0.0F,
  0.0F, -1.0F, 0.0F, 0.0F,
  0.0, 0.0, 0.0, 1
  );
applyMatrix(
  1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz),     2 * (qx * qz + qw * qy),     0,
  2 * (qx * qy + qw * qz),     1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx),     0,
  2 * (qx * qz - qw * qy),     2 * (qy * qz + qw * qx),     1 - 2 * (qx * qx + qy * qy), 0,
  0,                           0,                           0,                           1
);
  applyMatrix(
  1.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 1.0F, 0.0F,
  0.0F, -1.0F, 0.0F, 0.0F,
  0.0, 0.0, 0.0, 1
  );



/*applyMatrix(1, 2*(qx*qy - qw*qz), 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1);*/
  pushMatrix();

 noStroke();
  model.draw();
  popMatrix();
  popMatrix();
  //println("draw");
}
boolean isReading = false;
int type, dataSize, bytesRead;
byte[] dataBuffer;

void serialEvent(Serial myPort) {
    while (myPort.available() > 0) {
        int incomingByte = myPort.read() & 0xFF;  // Read as unsigned byte
        
        if (!isReading) {
            // Looking for start byte
            if (incomingByte == MSG_START) {
                isReading = true;
                bytesRead = 0;
                dataBuffer = null;
            }
        } else if (bytesRead == 0) {
            // Read packet type
            type = incomingByte;
            bytesRead++;
        } else if (bytesRead == 1) {
            // Read data size
            dataSize = incomingByte;
            dataBuffer = new byte[dataSize];
            bytesRead++;
        } else if (bytesRead - 2 < dataSize) {
            // Read data into buffer
            dataBuffer[bytesRead - 2] = (byte) incomingByte;
            bytesRead++;
            
            // If all data received, process the packet
            if (bytesRead - 2 == dataSize) {
                processMessage(type, dataBuffer);
                isReading = false;  // Reset for next packet
            }
        }
    }
}

void processMessage(int type, byte[] data) {
    println("Received Type: " + type + ", Data Size: " + data.length);

    // Convert byte data to float if applicable
    if (data.length % 4 == 0) {
        float[] floatData = new float[data.length / 4];
        for (int i = 0; i < floatData.length; i++) {
            int intBits = ((data[i * 4] & 0xFF) << 24) | 
                          ((data[i * 4 + 1] & 0xFF) << 16) |
                          ((data[i * 4 + 2] & 0xFF) << 8) |
                          ((data[i * 4 + 3] & 0xFF));
            floatData[i] = Float.intBitsToFloat(intBits);
            println(Float.toHexString(floatData[i]));
        }
        ax = floatData[2];
        ay = floatData[3];
        az = floatData[4];
        
        qw = floatData[20];
        qx = floatData[21];
        qy = floatData[22];
        qz = floatData[23];
      //  println("Float Data: " + join(str(floatToHexString\\\(floatData)), ", "));
    } else {
        println("Raw Data: " + new String(data));  // Print as string if not float
    }
}
/*
void serialEvent(Serial p) 
{
  byte[] incoming = new byte[256];

  
 // p.readBytes(incoming);
  int incoming_byte = p.read();
  
  println(Integer.toHexString(incoming_byte));

  //if (printSerial) {
   //println(incoming);
 // }
   
   
   msg_type = incoming[1];
   msg_size = incoming[2];
   
   byte[] result = new byte[30];
  float[] telemArray = byteArrayToFloatArray(Arrays.copyOfRange(incoming, 3, msg_size+3-1));
  if ((telemArray.length == 31))
  {
    println(telemArray);
    
    
    if (incoming[0] == MSG_START)
    {

      
     // buffer = incoming;
     // println(roll);
     // println(pitch);
     //println(yaw);
     
     time = telemArray[0];
     
     System.out.println(time);
     
     ax = telemArray[1];
     ay = telemArray[2];
     ay = telemArray[3];
     
     ax_local = telemArray[4];
     ay_local = telemArray[5];
     az_local = telemArray[6];
     
     vx = telemArray[10];
     vy = telemArray[11];
     vz = telemArray[12];
     
     vx_local = telemArray[13];
     vy_local = telemArray[14];
     vz_local = telemArray[15];
     
     qw = telemArray[19];
     qx = telemArray[20];
     qy = telemArray[21];
     qz = telemArray[22];
     
     alt = telemArray[27];
    }
    /*if (incoming.startsWith("Acceleration")){
      String[] list = split(incoming, ":")[1].split(",");
      println(list[0]);
    println(list[1]);
      println(list[2]);
    
     ax = float(list[0].trim());
     ay = float(list[1].trim());
     az = float(list[2].trim());
      
      
      buffer = incoming;
    }
    


  }
}*/

// Set serial port to desired value.
void setSerialPort(String portName) {
  // Close the port if it's currently open.
  if (port != null) {
    port.stop();
  }
  try {
    // Open port.
    port = new Serial(this, portName, 115200);
    port.bufferUntil('\n');
    // Persist port in configuration.
    saveStrings(serialConfigFile, new String[] { portName });
  }
  catch (RuntimeException ex) {
    // Swallow error if port can't be opened, keep port closed.
    port = null; 
  }
}

// UI event handlers

void handlePanelEvents(GPanel panel, GEvent event) {
  // Panel events, do nothing.
}

void handleDropListEvents(GDropList list, GEvent event) { 
  // Drop list events, check if new serial port is selected.
  if (list == serialList) {
    setSerialPort(serialList.getSelectedText()); 
  }
}

void handleToggleControlEvents(GToggleControl checkbox, GEvent event) { 
  // Checkbox toggle events, check if print events is toggled.
  if (checkbox == printSerialCheckbox) {
    printSerial = printSerialCheckbox.isSelected(); 
  }
}

public static float[] byteArrayToFloatArray(byte[] bytes) {
        int startIndex = 3;
        int endIndex = bytes.length - 1;
        if (bytes.length % 4 != 0) {
            throw new IllegalArgumentException("Byte array length must be a multiple of 4");
        }

        FloatBuffer floatBuffer = ByteBuffer.wrap(bytes, startIndex, endIndex-startIndex)
                                            .order(ByteOrder.LITTLE_ENDIAN) // Adjust if needed
                                            .asFloatBuffer();

        float[] floats = new float[floatBuffer.remaining()];
        floatBuffer.get(floats);
        return floats;
    }
