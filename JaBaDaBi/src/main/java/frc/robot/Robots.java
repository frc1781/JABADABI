package frc.robot;

public enum Robots {
    DE_RONNA("deRonna"),
    SAVITAR("savitara"),
    AVA("ava"),
    RALPH("ralph"),
    NOBODY("nobody");

    private final String name;

    Robots(String name) {
        this.name = name;
    }

    public String asString() {
        return name;
    }

    public static Robots fromString(String s) {
        return switch (s) {
            case "deRonna" -> DE_RONNA;
            case "savitara" -> SAVITAR;
            case "ava" -> AVA;
            case "ralph" -> RALPH;
            default -> NOBODY;
        };
    }
}
