package self.lesfoster.dht;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class DhtUtilsTest {
    @Test
    void testTranslateDht22() {
        Assertions.assertEquals("  31.00% RH, 24.00 C", DhtUtils.translateDht22("013600F000"));
    }

    @Test
    void testTranslateDht11() {
        Assertions.assertEquals("  37.00% RH, 22.00 C", DhtUtils.translateDht11("2500160100"));
    }
}
