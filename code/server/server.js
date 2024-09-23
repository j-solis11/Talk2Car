const express = require('express');
const dgram = require('dgram');
const bodyParser = require('body-parser');
const cors = require('cors');
const app = express();

msgToEsp = Buffer.from('0');
const udpServer = dgram.createSocket('udp4');
let new_message = "-1"
let old_message = "blank"

// Use bodyParser middleware to parse JSON bodies
app.use(bodyParser.json());
app.use(cors());

// Handle POST requests from the React app
app.post('/data', (req, res) => {
    new_message = req.body.message;
    console.log(new_message);
    res.status(200).send('Data received');
});

udpServer.on('message', (msg, rinfo) => {
    console.log(`Received UDP message from ${rinfo.address}:${rinfo.port}: ${msg}`);
    if (new_message != old_message) {
        msgToEsp = Buffer.from(new_message);
        console.log(`sending ${new_message}`);
    }
    else {
        msgToEsp = Buffer.from('-1');
    }
    udpServer.send(msgToEsp, rinfo.port, rinfo.address);
    old_message = new_message;
});

app.get('/', (req, res) => {
    const commands = ['slowing down', 'speeding up', 'stopping', 'turning left', 'turning right', 'turning forward'];
    let command = commands[new_message];
    res.send(`
        <!DOCTYPE html>
        <html lang="en">
        <head>
            <meta charset="UTF-8">
            <title>Server Data</title>
            <script>
                setInterval(function() {
                    window.location.reload();
                }, 250); // 1000 milliseconds = 1 second
            </script>
        </head>
        <body style="text-align:center">
            <h2>Available Commands: Slow, Fast, Stop, Left, Right, Straight</h2>
            <br>
            <h1>${command}</h1>
        </body>
        </html>
    `);
});

udpServer.on('error', (err) => {
    console.error(`UDP server error:\n${err.stack}`);
    udpServer.close();
});

udpServer.on('listening', () => {
    const address = udpServer.address();
    console.log(`UDP server listening on ${address.address}:${address.port}`);
});
  
udpServer.bind(3333); // UDP port to listen on

app.listen(8000, () => {
    console.log('Server running on port 8000');
});