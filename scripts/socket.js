var PORT = 9876;
var HOST = '192.168.169.1';

var dgram = require('dgram');
var message = Buffer.from('My KungFu is Good!');

var client = dgram.createSocket('udp4');

client.addListener("message", (msg) =>
{
    console.log(msg.toString());
    client.close();
})
client.send(message, PORT, HOST, function(err, bytesCount) {
    if (err) throw err;
    console.log('UDP message sent to ' + HOST + ':' + PORT, bytesCount);
});