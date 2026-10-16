package org.firstinspires.ftc.teamcode.consts;

public enum Bots {
    PRIMARY("23644-A-RC"),
    SECONDARY("23644-B-RC");

    private final String ssid;

    Bots(String ssid) {
        this.ssid = ssid;
    }

    public String getSsid() {
        return ssid;
    }

    public static Bots ssidToBot(String ssid) {
        for (Bots bot : Bots.values()) {
            if (bot.getSsid().equals(ssid)) {
                return bot;
            }
        }
        return null;
    }
}
