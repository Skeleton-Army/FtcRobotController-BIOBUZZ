package org.firstinspires.ftc.teamcode.consts;

import androidx.annotation.NonNull;

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

    @NonNull
    public static Bots ssidToBot(String ssid) {
        ssid = ssid.replace("\"", "");
        for (Bots bot : Bots.values()) {
            if (bot.getSsid().equalsIgnoreCase(ssid)) {
                return bot;
            }
        }
        return PRIMARY;
    }
}
