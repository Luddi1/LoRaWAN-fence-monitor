function fence_decoder(bytes, port) {
    var decoded = {};

    decoded.voltage = (bytes[0] & 0x7F) / 10;

    decoded.battery_empty = (bytes[0] >> 7) & 0x01;

    return decoded;
}

function Decoder(bytes, port) {

    if (port == 2) {
        return fence_decoder(bytes, port);
    }

}