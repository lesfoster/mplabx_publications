package self.lesfoster.dht;

public class DhtUtils {
    static String translateDht22(String reading) {
        double rh = Integer.decode("0x" + reading.substring(0,4)) / 10.0;
        double temp = Integer.decode("0x" + reading.substring(4,8)) / 10.0;
        return String.format("%7.2f%% RH, %5.2f C", rh, temp);
    }

    static String translateDht11(String reading) {
        double rh = Integer.decode("0x" + reading.substring(0, 2)) + Integer.decode("0x" + reading.substring(2, 4))/255.0;
        double temp = Integer.decode("0x" + reading.substring(4, 6)) + Integer.decode("0x" + reading.substring(6, 8))/255.0;
        return String.format("%7.2f%% RH, %5.2f C", rh, temp);
    }
}
