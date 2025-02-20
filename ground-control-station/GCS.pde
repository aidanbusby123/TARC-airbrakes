import processing.serial.*;
import java.awt.datatransfer.*;
import java.awt.Toolkit;
import processing.opengl.*;
import saito.objloader.*;
import g4p_controls.*;

float roll  = 0.0F;
float pitch = 0.0F;
float yaw   = 0.0F;
float temp  = 0.0F;
float alt   = 0.0F;

float qw = 0.0F;
float qx = 0.0F;
float qy = 0.0F;
float qz = 0.0F;

float ax = 0.0F;
float ay = 0.0F;
float az = 0.0F;

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
GLabel accel_x_Label, accel_y_Label, accel_z_Label, gyro_x_Label, gyro_y_Label, gyro_z_Label;

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
  int selectedPort = 0;
  String[] availablePorts = Serial.list();
  if (availablePorts == null) {
    println("ERROR: No serial ports available!");
    exit();
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
  // Build serial config UI.
  configPanel = new GPanel(this, 10, 10, width-20, 90, "Configuration (click to hide/show)");
  serialLabel = new GLabel(this,  0, 20, 80, 25, "Serial port:");
  configPanel.addControl(serialLabel);
  serialList = new GDropList(this, 90, 20, 200, 200, 6);
  serialList.setItems(availablePorts, selectedPort);
  configPanel.addControl(serialList);
  printSerialCheckbox = new GCheckbox(this, 5, 50, 200, 20, "Print serial data");
  printSerialCheckbox.setSelected(printSerial);
  configPanel.addControl(printSerialCheckbox);
  
  dataPanel = new GPanel(this, 10, 110, 380, 200, "Telemetry");
  dataPanel.setCollapsed(false);
  
  accel_x_Label = new GLabel(this, 10, 20, 200, 20, "AX: 0.0");
  accel_y_Label = new GLabel(this, 10, 50, 200, 20, "AY: 0.0");
  accel_z_Label = new GLabel(this, 10, 80, 200, 20, "AZ: 0.0");
  
  dataPanel.addControl(accel_x_Label);
  dataPanel.addControl(accel_y_Label);
  dataPanel.addControl(accel_z_Label);
  
  
  
  // Set serial port.
 setSerialPort(serialList.getSelectedText());
}
 
void draw()
{
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

void serialEvent(Serial p) 
{
  String incoming = p.readString();
  //if (printSerial) {
   //println(incoming);
 // }
  
  if ((incoming.length() > 8))
  {
    

    if (incoming.startsWith("Quaternion"))
    {
      String[] list = split(incoming, ":")[1].split(",");
      println(list[0]);
    println(list[1]);
      println(list[2]);
      println(list[3]);
    
      roll  = float(list[1].replace(',', ' '));
      pitch = float(list[2].replace(',', ' '));
      yaw   = float(list[3]);
      
     qw = float(list[0].trim());
     qx = float(list[1].trim());
     qy = float(list[2].trim());
     qz = float(list[3].trim());
     // println(qw);
      
      
      buffer = incoming;
     // println(roll);
     // println(pitch);
     //println(yaw);
    }
    if (incoming.startsWith("Acceleration")){
      String[] list = split(incoming, ":")[1].split(",");
      println(list[0]);
    println(list[1]);
      println(list[2]);
    
     ax = float(list[0].trim());
     ay = float(list[1].trim());
     az = float(list[2].trim());
      
      
      buffer = incoming;
    }
    /*
    ax = float(list[6].replace(',', ' '));
    ay = float(list[7].replace(',', ' '));
    az = float(list[8].replace(',', ' '));
    */

  }
}

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
