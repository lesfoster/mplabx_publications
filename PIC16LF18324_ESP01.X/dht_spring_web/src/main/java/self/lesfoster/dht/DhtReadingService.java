package self.lesfoster.dht;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.ResponseBody;
import org.springframework.web.bind.annotation.RestController;

import static self.lesfoster.dht.DhtUtils.translateDht11;
import static self.lesfoster.dht.DhtUtils.translateDht22;

@RestController
public class DhtReadingService {
    private static final Logger LOGGER = LoggerFactory.getLogger(DhtReadingService.class);
    @GetMapping("/reading/{reading}")
    @ResponseBody
    public String getReading(@PathVariable String reading) {
        LOGGER.info(reading);
        if (reading.charAt(reading.length() - 1) == '1') {
            LOGGER.warn("CRC check failed");
        }
        if (reading.charAt(0) > '1') {
            LOGGER.info(translateDht11(reading) + " as DHT11");
        } else {
            LOGGER.info(translateDht22(reading) + " as DHT22");
        }

        return "OK";
    }


}
