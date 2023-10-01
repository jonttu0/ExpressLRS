#pragma once
#include <pgmspace.h>

static const char PROGMEM GO_BACK[] = R"rawliteral(
<!DOCTYPE html><html><head></head>
<body><script>javascript:history.back();</script></body>
</html>
)rawliteral";

static const char PROGMEM RECOVER_HTML[] = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0">
<title>ExpressLRS</title>
<style>body {background-color: #1E1E1E;font-family: Arial, Helvetica, Sans-Serif;Color: #69cbf7;}</style>
</head><body onload="javascript:start();">
Page not found! Open <a href="/update">/update</a> for recovery...<br/>
</body></html>
)rawliteral";
