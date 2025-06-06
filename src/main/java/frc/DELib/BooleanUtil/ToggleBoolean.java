package frc.DELib.BooleanUtil;

public class ToggleBoolean {
    private boolean released = true;
    private boolean retVal = false;

    public boolean update(boolean newValue) {
        if(newValue && released) {
            released = false;
            retVal = !retVal;
        }
        if(!newValue) {
            released = true;
        }
        return retVal;
    }
}
