const express = require('express');
const http = require('http');
const WebSocket = require('ws');

const app = express();
app.use(express.static('public'));
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

setInterval(() => {
  const data = {
    time: new Date().toISOString(),
    message: "Hello from Jetson Nano !"
  };

  wss.clients.forEach(client => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(JSON.stringify(data));
    }
  });
}, 1000);

const PORT = 3000;
server.listen(PORT, '0.0.0.0', () => {
  console.log(`Server running on port ${PORT}`);
});
