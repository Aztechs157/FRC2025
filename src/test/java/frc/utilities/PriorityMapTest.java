package frc.utilities;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class PriorityMapTest {

    private PriorityMap<String, String> map = new PriorityMap<String, String>();

    @BeforeEach
    void setup() {
        map.clear();
    }

    @Test
    void basicFunctions() {
        assertNull(map.put("hello", 2, "world"));
        assertNull(map.put("my friend", 0, "is nice"));

        assertEquals(2, map.size());

        assertEquals(2, map.getPriority("hello"));
        assertEquals(0, map.getPriority("my friend"));

        assertEquals("is nice", map.firstValue());
        assertEquals("my friend", map.firstKey());
        assertEquals("world", map.lastValue());
        assertEquals("hello", map.lastKey());

        assertEquals("world", map.get("hello"));
        assertEquals("is nice", map.get("my friend"));

        assertEquals("is nice", map.remove("my friend"));
        assertEquals(1, map.size());
        assertNull(map.remove("my friend"));
        assertEquals(1, map.size());
        assertNull(map.get("my friend"));

        assertEquals("world", map.get("hello"));
        assertEquals("world", map.firstValue());
        assertEquals("world", map.lastValue());
        assertEquals("hello", map.firstKey());
        assertEquals("hello", map.lastKey());

        assertEquals("world", map.remove("hello"));
        assertEquals(0, map.size());
        assertNull(map.remove("hello"));
        assertEquals(0, map.size());
        assertNull(map.get("hello"));

        assertNull(map.firstValue());
        assertNull(map.lastValue());
        assertNull(map.firstKey());
        assertNull(map.lastKey());
    }

    @Test
    void manyValues() {
        map.put("h", 0, "h");
        map.put("g", 1, "g");
        map.put("f", 2, "f");
        map.put("e", 3, "e");
        map.put("d", 4, "d");
        map.put("c", 5, "c");
        map.put("b", 6, "b");
        map.put("a", 7, "a");

        assertEquals(8, map.size());

        assertEquals("a", map.get("a"));
        assertEquals("b", map.get("b"));
        assertEquals("c", map.get("c"));
        assertEquals("d", map.get("d"));
        assertEquals("e", map.get("e"));
        assertEquals("f", map.get("f"));
        assertEquals("g", map.get("g"));
        assertEquals("h", map.get("h"));

        assertEquals("h", map.firstValue());
        assertEquals("h", map.firstKey());
        assertEquals("a", map.lastValue());
        assertEquals("a", map.lastKey());

        map.remove("h");
        assertEquals(7, map.size());
        map.remove("a");
        assertEquals(6, map.size());

        assertEquals("g", map.firstValue());
        assertEquals("g", map.firstKey());
        assertEquals("b", map.lastValue());
        assertEquals("b", map.lastKey());

    }

    @Test
    void replaceTest() {
        map.put("some key", 5, "some value");

        assertEquals("some value", map.get("some key"));
        assertEquals(5, map.getPriority("some key"));

        map.replace("some key", "a second value");

        assertEquals("a second value", map.get("some key"));
        assertEquals(5, map.getPriority("some key"));

        map.replace("some key", 10, "a third value");

        assertEquals("a third value", map.get("some key"));
        assertEquals(10, map.getPriority("some key"));
    }
}
