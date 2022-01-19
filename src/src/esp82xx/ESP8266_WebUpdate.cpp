#include "debug_elrs.h"
#include "targets.h"

//#define STATIC_IP_AP     "192.168.4.1"
//#define STATIC_IP_CLIENT "192.168.1.50"

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>

#if WIFI_MANAGER
#include <WiFiManager.h>
#endif /* WIFI_MANAGER */

#ifdef WIFI_AP_SSID
#define STASSID WIFI_AP_SSID
#else
#define STASSID "ExpressLRS AP"
#endif
#ifdef WIFI_AP_PSK
#define STAPSK WIFI_AP_PSK
#else
#define STAPSK "expresslrs"
#endif

#ifndef WIFI_TIMEOUT
#define WIFI_TIMEOUT 60 // default to 1min
#endif


MDNSResponder MDNS;
ESP8266WebServer httpServer(SERVER_PORT);
ESP8266HTTPUpdateServer httpUpdater;

void BeginWebUpdate(void)
{
    const char *host = "elrs_rx";
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("Begin Webupdater");
#endif

    wifi_station_set_hostname(host);

    IPAddress addr;

    WiFi.persistent(false);
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    WiFi.setOutputPower(13);
    WiFi.setPhyMode(WIFI_PHY_MODE_11N);

#if defined(WIFI_SSID) && defined(WIFI_PSK)
    if (WiFi.status() != WL_CONNECTED) {
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PSK);
    }
    uint32_t i = 0;

    #define TIMEOUT (WIFI_TIMEOUT * 10)
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(100);
        if (++i > TIMEOUT) {
        break;
        }
    }
    if (WiFi.status() == WL_CONNECTED) {
        addr = WiFi.localIP();
    } else
#elif WIFI_MANAGER
    WiFiManager wifiManager;
    //WiFiManagerParameter header("<p>Express LRS ESP82xx RX</p>");
    //wifiManager.addParameter(&header);

#ifdef STATIC_IP_AP
    /* Static portal IP (default: 192.168.4.1) */
    addr.fromString(STATIC_IP_CLIENT);
    wifiManager.setAPStaticIPConfig(addr,
                                    IPAddress(192,168,4,1),
                                    IPAddress(255,255,255,0));
#endif /* STATIC_IP_AP */
#ifdef STATIC_IP_CLIENT
    /* Static client IP */
    addr.fromString(STATIC_IP_CLIENT);
    wifiManager.setSTAStaticIPConfig(addr,
                                     IPAddress(192,168,1,1),
                                     IPAddress(255,255,255,0));
#endif /* STATIC_IP_CLIENT */

    wifiManager.setConfigPortalTimeout(WIFI_TIMEOUT);
    if (wifiManager.autoConnect(STASSID" ESP RX")) // start unsecure portal AP
    {
        addr = WiFi.localIP();
    }
    else
#endif /* WIFI_MANAGER */
    {
        // No wifi found, start AP
        WiFi.mode(WIFI_OFF);
        delay(1000);
        WiFi.mode(WIFI_AP);
        WiFi.softAP(STASSID" ESP RX", STAPSK);
        addr = WiFi.softAPIP();
    }

    MDNS.end();
    if (MDNS.begin(host, addr))
    {
        String instance = String(host) + "_" + WiFi.macAddress();
        instance.replace(":", "");
        MDNS.setInstanceName(host);
        MDNSResponder::hMDNSService service = MDNS.addService(instance.c_str(), "http", "tcp", SERVER_PORT);
        MDNS.addServiceTxt(service, "vendor", "elrs");
#ifdef TARGET_INDENTIFIER
        MDNS.addServiceTxt(service, "target", TARGET_INDENTIFIER);
#else
        MDNS.addServiceTxt(service, "target", "ESP82xx Generic RX");
#endif
        MDNS.addServiceTxt(service, "version", LATEST_COMMIT_STR);
        MDNS.addServiceTxt(service, "type", "rx");

        // If the probe result fails because there is another device on the network with the same name
        // use our unique instance name as the hostname. A better way to do this would be to use
        // MDNSResponder::indexDomain and change wifi_hostname as well.
        MDNS.setHostProbeResultCallback([instance](const char* p_pcDomainName, bool p_bProbeResult) {
            if (!p_bProbeResult) {
                WiFi.hostname(instance);
                MDNS.setInstanceName(instance);
            }
        });
    }

    httpUpdater.setup(&httpServer);
    httpServer.begin();
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.printf("HTTPUpdateServer ready! Open http://%s.local/update in your browser\n", host);
#endif
}

void HandleWebUpdate(void)
{
    httpServer.handleClient();
    MDNS.update();
}
