

#include <Arduino_BuiltIn.h>
#include <string.h>
#include "calibration.h"
#ifdef BNO085
#include "BNO085.h"
#else
#include "Cmps14.h"
#endif

#define FSYS LittleFS

#define MaxHeaderLength 16    //maximum length of http header required
#define SENDTEXT(X) webServer.send(200,ct_plaintext,X)
#define SENDHTML(X) webServer.send(200,ct_html,X)
#define SENDJSON(X) webServer.send(200,ct_json,X)
#define SENDJSCRIPT(X) webServer.send(200,ct_javascript,X)
// TRACE output simplified, can be deactivated here
#define TRACE(...) Serial.printf(__VA_ARGS__)

extern WiFiClient configClient, webClient;
extern Preferences settings;
extern unsigned short sensorHeading, boatHeading;

String HttpHeader = String(MaxHeaderLength);

// We need to specify some content-type mapping, so the resources get delivered with the
// right content type and are displayed correctly in the browser

const char ct_plaintext[16] = {"text/plain"};
const char ct_html[32] = {"text/html; charset=\"UTF-8\""};
const char ct_json[32] = {"application/json"};
const char ct_javascript[32] = {"application/javascript"};

char contentTypes[][2][32] =
{
  {".txt", "text/plain"},
  {".png",  "image/png"},
  {".jpg",  "image/jpg"},
  {"", ""}
};

//the WebServer is created in the main .ino
extern WebServer webServer;

// The FileServerHandler is registered to the web server to support DELETE and UPLOAD of files into the filesystem.
class FileServerHandler : public RequestHandler {
public:
  // @brief Construct a new File Server Handler object
  // @param fs The file system to be used.
  // @param path Path to the root folder in the file system that is used for serving static data down and upload.
  // @param cache_header Cache Header to be used in replies.
  FileServerHandler() {
    TRACE("FileServerHandler is registered\n");
  }

  // @brief check incoming request. Can handle POST for uploads and DELETE.
  // @param requestMethod method of the http request line.
  // @param requestUri request resource from the http request line.
  // @return true when method can be handled.
  bool canHandle(WebServer &server, HTTPMethod requestMethod, const String &uri) override {
    return ((requestMethod == HTTP_POST) || (requestMethod == HTTP_DELETE));
  }  // canHandle()

  bool canUpload(WebServer &server, const String &uri) override {
    // only allow upload on root fs level.
    return (true);
  }  // canUpload()

  bool handle(WebServer &server, HTTPMethod requestMethod, const String &requestUri) override {
    // ensure that filename starts with '/'
    String fName = requestUri;
    if (!fName.startsWith("/")) {
      fName = "/" + fName;
    }

    if (requestMethod == HTTP_POST) {
      // all done in upload. no other forms.

    } else if (requestMethod == HTTP_DELETE) {
      if (FSYS.exists(fName)) {
        TRACE("DELETE %s\n", fName.c_str());
        FSYS.remove(fName);
      }
    }  // if

    server.send(200);  // all done.
    return (true);
  }  // handle()

  // uploading process
  void upload(WebServer &webServer, const String &requestUri, HTTPUpload &upload) override {
    TRACE("FileServerHandler - upload() called");
    // ensure that filename starts with '/'
    static size_t uploadSize;
    if (upload.status == UPLOAD_FILE_START) {
      String fName = upload.filename;
      TRACE("UPLOAD %s\n", fName.c_str());
      // Open the file for writing
      if (!fName.startsWith("/")) {
        fName = "/" + fName;
      }
      TRACE("start uploading file %s...\n", fName.c_str());

      if (FSYS.exists(fName)) {
        FSYS.remove(fName);
      }  // if
      _fsUploadFile = FSYS.open(fName, "w");
      uploadSize = 0;

    } else if (upload.status == UPLOAD_FILE_WRITE) {
      // Write received bytes
      if (_fsUploadFile) {
        size_t written = _fsUploadFile.write(upload.buf, upload.currentSize);
        if (written < upload.currentSize) {
          // upload failed
          TRACE("  write error!\n");
          _fsUploadFile.close();

          // delete file to free up space in filesystem
          String fName = upload.filename;
          if (!fName.startsWith("/")) {
            fName = "/" + fName;
          }
          FSYS.remove(fName);
        }
        uploadSize += upload.currentSize;
        // TRACE("free:: %d of %d\n", LittleFS.usedBytes(), LittleFS.totalBytes());
        // TRACE("written:: %d of %d\n", written, upload.currentSize);
        // TRACE("totalSize: %d\n", upload.currentSize + upload.totalSize);
      }  // if

    } else if (upload.status == UPLOAD_FILE_END) {
      TRACE("upload done.\n");
      // Close the file
      if (_fsUploadFile) {
        _fsUploadFile.close();
        TRACE(" %d bytes uploaded.\n", upload.totalSize);
      }
    }  else TRACE("upload called with invalid status.\n");

  }  // upload()

protected:
  File _fsUploadFile;
};

// Declare some handler functions for the various URLs on the server


void handleFormUpload();
void handleDirectory();
void handleGetCalStatus();
void handleDisableCalibration();
void handleEnableGyroCalib();
void handleEnableAccelCalib();
void handleEnableMagCalib();
void handleSaveCalibration();
void handleResetCalibration();
void handleGetHeading();
void handleSaveCard();
void handleGenerateCard();
void handleResetCard();


std::string htmlEncode(std::string data)
{
  // Quick and dirty: doesn't handle control chars and such.
  const char *p = data.c_str();
  std::string rv = "";

  while (p && *p)
  {
    char escapeChar = *p++;

    switch (escapeChar)
    {
      case '&':
        rv += "&amp;";
        break;

      case '<':
        rv += "&lt;";
        break;

      case '>':
        rv += "&gt;";
        break;

      case '"':
        rv += "&quot;";
        break;

      case '\'':
        rv += "&#x27;";
        break;

      case '/':
        rv += "&#x2F;";
        break;

      default:
        rv += escapeChar;
        break;
    }
  }

  return rv;
}



//Setup our webserver  
void webServerSetup() {
// associate the various server paths to handler functions
 // webServer.on("/upload",handleFormUpload);
  // serve a built-in htm page
  webServer.on("/$upload.htm", handleFormUpload);
  webServer.on("/",handleDirectory);
  webServer.on("/getCalStatus",handleGetCalStatus);
  webServer.on("/generateCard",handleGenerateCard);
  webServer.on("/disableCal",handleDisableCalibration);
  webServer.on("/enableGyroCal",handleEnableGyroCalib);
  webServer.on("/enableAccelCal",handleEnableAccelCalib);
  webServer.on("/enableMagCal",handleEnableMagCalib);
  webServer.on("/saveCal",handleSaveCalibration);
  webServer.on("/resetCal",handleResetCalibration);
  webServer.on("/getHeading",handleGetHeading);
  webServer.on("/saveCard",handleSaveCard);
  webServer.on("/generateCard",handleGenerateCard);
  webServer.on("/resetCard",handleResetCard);


  Serial.println("Starting web server...");
  webServer.addHandler(new FileServerHandler()); //Handles file upload
  webServer.enableCORS(true); //Safe in this situation
  webServer.serveStatic("/", FSYS, "/"); //Allows access to all files on FSYS
  webServer.onNotFound([]() {
    // standard not found in browser.
    webServer.send(404, "text/html", FPSTR("File Not Found"));
  });
  webServer.enableETag(true);
  webServer.begin();
}

//This little procedure puts the CMPS14 into configuration mode
//Note that it is up to the calling function to call endTransmission(), after it has sent the actual
//configuration commands. This procedure just sets things up


/*
 All of these "handle..." functions below handle all of the relevant REST API calls
 */ 

 //Linked to URL /upload
 //Sends some html/javascript to the client that allows the user to drag and drop a file 
 //to be uploaded. 
void handleFormUpload()
{
  static const char uploadContent[] PROGMEM =
  R"==(
<!doctype html>
<html lang='en'>

<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Upload</title>
</head>

<body style="width:300px">
  <h1>Upload</h1>
  <div><a href="/">Home</a></div>
  <hr>
  <div id='zone' style='width:16em;height:12em;padding:10px;background-color:#ddd'>Drop file here...</div>

  <script>
    // allow drag&drop of file objects
    function dragHelper(e) {
      e.stopPropagation();
      e.preventDefault();
    }

    // allow drag&drop of file objects
    function dropped(e) {
      dragHelper(e);
      var fls = e.dataTransfer.files;
      var formData = new FormData();
      for (var i = 0; i < fls.length; i++) {
        formData.append('file', fls[i], '/' + fls[i].name);
      }
      fetch('/public/', { method: 'POST', body: formData }).then(function () {
        window.alert('done.');
      });
    }

    var z = document.getElementById('zone');
    z.addEventListener('dragenter', dragHelper, false);
    z.addEventListener('dragover', dragHelper, false);
    z.addEventListener('drop', dropped, false);
  </script>
</body>
)==";

// used for $upload.htm
static const char notFoundContent[] PROGMEM = R"==(
<html>
<head>
  <title>Resource not found</title>
</head>
<body>
  <p>The resource was not found.</p>
  <p><a href="/">Start again</a></p>
</body>
)==";

//  webServer.sendHeader("Content-Type", "text/html");
  TRACE("HandleFormUpload Called\n");
  SENDHTML(uploadContent);  

}

//Provides a FSYS file listing 
void handleDirectory()
{
  TRACE("handleDirectory called.\n");
  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  SENDHTML("<html><head><title>File Listing</title></head><body>");
  File d = FSYS.open("/");

  char buff[256];

  if (!d.isDirectory())
  {
    TRACE("Could not open FSYS root directory.\n");
    SENDHTML("<p>No files found.</p>");
  }
  else
  {
    TRACE("Listing Files\n");
    SENDHTML("<div><p><u>File Listing</u></p></div>");
    SENDHTML("<ol>");
    File f = d.openNextFile();

    while (f)
    {
      std::string pathname(f.name());
      sprintf(buff,"<li><a href=\"%s\">%s</a></li>", pathname.c_str(), pathname.c_str());
      SENDHTML(buff);
/*
      if (pathname.rfind(".txt") != std::string::npos)
      {
//        std::string filename = pathname.substr(8); // Remove /public/
//        sprintf(buff," <a href=\"/edit?filename=%s\">[edit]</a>", filename.c_str());
        sprintf(buff," <a href=\"filename=%s\"></a>", pathname.c_str());
        SENDHTML(buff);
      }

      SENDHTML("</li>");
*/
      f = d.openNextFile();
    }

    SENDHTML("</ol>");
  }

  SENDHTML("</body>");
}

void handleGetCalStatus()
{
  byte calStatus;
  char buff[128];
  Serial.println("HandleGetCalStaus() Called");

  calStatus = getCalibration();
  byte sys = (calStatus & 0b11000000) >> 6;
  byte gyro = (calStatus & 0b00110000) >> 4;
  byte accel = (calStatus & 0b00001100) >> 2;
  byte mag = (calStatus & 0b00000011);

  // Set content type of the response
  webServer.sendHeader("Access-Control-Allow-Origin", "*");

  // Write a JSON response
  sprintf(buff,"{\"sysStatus\":\"%d\",\"gyroStatus\":\"%d\",\"accelStatus\":\"%d\",\"magStatus\":\"%d\"}",sys,gyro,accel,mag);
  SENDJSON(buff);
 
}

void handleDisableCalibration()
{
  Serial.println("HandleDisableCalib() Called");
  disableCalibration();
  // Set content type of the response

  webServer.sendHeader("Access-Control-Allow-Origin", "*");

  // Write a JSON response
  SENDJSON("{ \"result\":\"OK\"}");
  
}

void handleEnableGyroCalib()
{
  Serial.println("HandleEnableGyroCalib() Called");
  enableGyroCalib();
  // Set content type of the response
  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  // Write a JSON response
  SENDJSON("{ \"result\":\"OK\"}");
}

void handleEnableAccelCalib()
{
  Serial.println("HandleEnableAccelCalib() Called");
  enableAccelerometerCalib();
  // Set content type of the response
  webServer.sendHeader("Access-Control-Allow-Origin", "*");

  // Write a JSON response
  SENDJSON("{ \"result\":\"OK\"}");
}

void handleEnableMagCalib()
{
  Serial.println("HandleEnableMagCalib() Called");
  enableMagnetometerCalib();

  // Set content type of the response
  webServer.sendHeader("Access-Control-Allow-Origin", "*");

  // Write a JSON response
  SENDJSON("{ \"result\":\"OK\"}");
}

void handleResetCalibration()
{
  Serial.println("HandleResetCalibration() Called");
  factoryReset();

  // Set content type of the resp
  webServer.sendHeader("Access-Control-Allow-Origin", "*");

  // Write a JSON response
  SENDJSON("{ \"result\":\"OK\"}");
}

void handleSaveCalibration()
{
  Serial.println("HandleSaveCalibration() Called");
  saveCalibration();  

  // Set content type of the resp
  webServer.sendHeader("Access-Control-Allow-Origin", "*");

  // Write a JSON response
  SENDJSON("{ \"result\":\"OK\"}");
}

//Returns current sensor heading
void handleGetHeading()
{
  char buff[128];
  Serial.println("handleGetHeading() Called");


  // Set content type of the response
  webServer.sendHeader("Access-Control-Allow-Origin", "*");

  // Write a JSON response 
  sprintf(buff,"{ \"result\":\"OK\",\"sensorHeading\":\"%03d\", \"boatHeading\":\"%03d\" }",sensorHeading,boatHeading);
  SENDJSON(buff);
}

//Generates a compass card from the supplied parameters
void handleGenerateCard()
{
  int north,south,east,west,count;
  String argval;
  char buff[16];

  Serial.println("handleGenerateCard() Called");

  //Extract named arguments from the  URL
  argval = webServer.arg("north");
  sprintf(buff, "%s", argval); //Kludge - convert String to string
  north = std::stoi(buff);

  argval = webServer.arg("east");
  sprintf(buff, "%s", argval); //Kludge - convert String to string
  east = std::stoi(buff);

  argval = webServer.arg("south");
  sprintf(buff, "%s", argval); //Kludge - convert String to string
  south = std::stoi(buff);

  argval = webServer.arg("west");
  sprintf(buff, "%s", argval); //Kludge - convert String to string
  west = std::stoi(buff);

  Serial.println("generateCard() called");

  calcOffsets(north,east,south,west);

  webServer.sendHeader("Access-Control-Allow-Origin", "*");

  SENDJSON("{ \"result\":\"OK\" }");
}
//Saves the compass card to NV ESP32 memory - is automatically reloaded on power-up
void handleSaveCard()
{
  Serial.println("handleSaveCard() Called");
  // Set content type of the response

  saveCompassCard();
  webServer.sendHeader("Access-Control-Allow-Origin", "*");

  // Write a JSON response 
  SENDJSON("{ \"result\":\"OK\" }");
}

void handleResetCard()
{
  Serial.println("handleZeroCard() Called");
  // Set content type of the response

  resetCompassCard();
  webServer.sendHeader("Access-Control-Allow-Origin", "*");

  // Write a JSON response 
  SENDJSON("{ \"result\":\"OK\" }");
}
