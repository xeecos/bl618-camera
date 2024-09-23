var PORT = 9876;
var HOST = '192.168.169.1';

var dgram = require('dgram');
var message = Buffer.from('My KungFu is Good!');

var client = dgram.createSocket('udp4');
let bufArray = [];
client.addListener("message", (msg) =>
{
    if (msg[0] == 0xff && msg[1] == 0xd8)
    {
        console.log("JPEG HEAD FOUND");
    }
    for (let i = 0; i < msg.byteLength; i++)
    {
        bufArray.push(msg[i]);
    }
    if (msg[msg.byteLength-2] == 0xff && msg[msg.byteLength-1] == 0xd9)
    {
        console.log("JPEG END FOUND");
    }
    console.log(bufArray.length);
});
client.send(message, PORT, HOST, function(err, bytesCount) {
    if (err) throw err;
    console.log('UDP message sent to ' + HOST + ':' + PORT, bytesCount);
});