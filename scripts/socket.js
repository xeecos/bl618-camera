const PORT = 9876;
const HOST = '192.168.169.1';
const fs = require("fs");
const dgram = require('dgram');
const message = Buffer.from('capture');
let timeout = 0;
let client = dgram.createSocket('udp4');
let bufArray = [];
let jpegLength = 0;
client.addListener("message", (msg) =>
{
    let jpegEnd = false;
    if (msg.byteLength < 64)
    {
        let str = msg.toString();
        if (str.indexOf("len:") > -1)
        {
            jpegLength = str.split("len:")[1] * 1;
            console.log("jpegLength:", jpegLength);
        }
        request();
        return;
    }
    for (let i = 0; i < msg.byteLength-1; i++)
    {
        if (msg[i] == 0xff && msg[i+1] == 0xd8)
        {
            console.log("JPEG HEAD FOUND:", i);
            bufArray = [];
            break;
        }
    }
    for (let i = 0; i < msg.byteLength; i++)
    {
        bufArray.push(msg[i]);
    }
    for (let i = 0; i < msg.byteLength-1; i++)
    {
        if (msg[i] == 0xff && msg[i+1] == 0xd9)
        {
            console.log("JPEG END FOUND:", i, msg.byteLength, bufArray.length);
            if (jpegLength == bufArray.length)
            {
                fs.writeFileSync("./tmp.jpg", Buffer.from(bufArray));
            }
            else
            {
                console.log("wrong jpeg length");
            }
            // console.timeEnd("capture");
            bufArray = [];
            jpegEnd = true;
            break;
        }
    }
    // if (bufArray.length > 0)
    {
        capture();
    }
    // if (jpegEnd)
    // {
    //     capture();
    // }
});
capture();
function capture ()
{
    clearTimeout(timeout);
    client.send(message, PORT, HOST, function(err, bytesCount) {
        if (err) throw err;
        // console.log('UDP message sent to ' + HOST + ':' + PORT, bytesCount);
        // console.time("capture");
        timeout = setTimeout(() =>
        {
            // console.timeEnd("capture");
            console.log("TIMEOUT!!!!!\n");
            capture();
        }, 60);
    });
}
function request ()
{
    client.send(Buffer.from(' '), PORT, HOST, function(err, bytesCount) {
        if (err) throw err;
    });
}