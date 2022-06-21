#if CONFIG_STM_UPDATER
#include "stm32_ota.h"
#include "main.h"
#include "stm32Updater.h"
#include "stk500.h"
#include <Arduino.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266WebServer.h>


extern ESP8266WebServer server;


File fsUploadFile; // a File object to temporarily store the received file
String uploadedfilename; // filename of uploaded file


static int8_t flash_stm32(uint32_t flash_addr)
{
  int8_t result = -1;
  websocket_send("STM32 Firmware Flash Requested!");
  websocket_send("  the firmware file: '" + uploadedfilename + "'");
  if (uploadedfilename.endsWith("firmware.elrs")) {
    result = stk500_write_file(uploadedfilename.c_str());
  } else if (uploadedfilename.endsWith("firmware.bin")) {
    result = esp8266_spifs_write_file(uploadedfilename.c_str(), flash_addr);
    if (result == 0)
      reset_stm32_to_app_mode(); // boot into app
  } else {
    websocket_send("Invalid file!");
  }
  Serial.begin(SERIAL_BAUD);
  return result;
}


void stm32_ota_handleFileUploadEnd()
{
  uint32_t flash_base = BEGIN_ADDRESS;
  //String message = "\nRequest params:\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    String name = server.argName(i);
    String value = server.arg(i);
      //message += " " + name + ": " + value + "\n";
      if (name == "flash_address") {
        flash_base = strtol(&value.c_str()[2], NULL, 16);
        break;
      }
  }
  //websocket_send(message);

  int8_t success = flash_stm32(flash_base);

  if (uploadedfilename.length() && FILESYSTEM.exists(uploadedfilename))
    FILESYSTEM.remove(uploadedfilename);

  server.sendHeader("Location", "/return");          // Redirect the client to the success page
  server.send(303);
  websocket_send(
    (success) ? "Update Successful!": "Update Failure!");
  server.send((success < 0) ? 400 : 200);
}


void stm32_ota_handleFileUpload()
{ // upload a new file to the SPIFFS
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START)
  {
    /* Remove old file */
    if (uploadedfilename.length() && FILESYSTEM.exists(uploadedfilename))
      FILESYSTEM.remove(uploadedfilename);

    FSInfo fs_info;
    if (FILESYSTEM.info(fs_info))
    {
      Dir dir = FILESYSTEM.openDir("/");
      while (dir.next()) {
        String file = dir.fileName();
        if (file.endsWith(".bin")) {
          FILESYSTEM.remove(file);
        }
      }

      String output = "Filesystem: used: ";
      output += fs_info.usedBytes;
      output += " / free: ";
      output += fs_info.totalBytes;
      websocket_send(output);

      if (fs_info.usedBytes > 0) {
        //websocket_send("formatting filesystem");
        //FILESYSTEM.format();
      }
    }
    else
    {
      websocket_send("SPIFFs Failed to init!");
      return;
    }
    uploadedfilename = upload.filename;

    websocket_send("Uploading file: " + uploadedfilename);

    if (!uploadedfilename.startsWith("/"))
    {
      uploadedfilename = "/" + uploadedfilename;
    }
    fsUploadFile = FILESYSTEM.open(uploadedfilename, "w"); // Open the file for writing in SPIFFS (create if it doesn't exist)
  }
  else if (upload.status == UPLOAD_FILE_WRITE)
  {
    if (fsUploadFile)
    {
      fsUploadFile.write(upload.buf, upload.currentSize); // Write the received bytes to the file
      String output = "Uploaded: ";
      output += fsUploadFile.position();
      output += " bytes";
      websocket_send(output);
    }
  }
  else if (upload.status == UPLOAD_FILE_END)
  {
    if (fsUploadFile)
    {                       // If the file was successfully created
      String totsize = "Total uploaded size ";
      totsize += fsUploadFile.position();
      totsize += " of ";
      totsize += upload.totalSize;
      websocket_send(totsize);
      server.send(100);
      fsUploadFile.close(); // Close the file again
    }
    else
    {
      server.send(500, "text/plain", "500: couldn't create file");
    }
  }
}
#endif // CONFIG_STM_UPDATER
