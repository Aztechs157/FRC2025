package frc.utilities;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonBox extends CommandGenericHID {

    public static enum ButtonBoxButtons {
        Bottom1(0),
        Bottom2(1),
        Bottom3(2),
        Bottom4(3),
        Bottom5(4),
        
        LeftTop(5),
        LeftBottom(6),
        
        MiddleTop(7),
        MiddleMid(8),
        MiddleBottom(9),

        PosSwitch(10),

        PosTopLeft(11),
        PosTopRight(12),
        PosMidLeft(13),
        PosMidRight(14),
        PosBottomLeft(15),
        PosBottomRight(16),
        PosBottomMid(17),
        PosPickupStation(18)

        ;

        public int value;
        
        ButtonBoxButtons(int value) {
            this.value = value;
        }
    }

    public ButtonBox(int port) {
        super(port);
    }

    public Trigger buttonBinding(ButtonBoxButtons button) {
        return button(button.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    // public Trigger Bottom1() {
    //     return buttonBinding(ButtonBoxButtons.Bottom1);
    // }
    // public Trigger Bottom2() {
    //     return buttonBinding(ButtonBoxButtons.Bottom2);
    // }
    // public Trigger Bottom3() {
    //     return buttonBinding(ButtonBoxButtons.Bottom3);
    // }
    // public Trigger Bottom4() {
    //     return buttonBinding(ButtonBoxButtons.Bottom4);
    // }
    // public Trigger Bottom5() {
    //     return buttonBinding(ButtonBoxButtons.Bottom5);
    // }
    // public Trigger LeftTop() {
    //     return buttonBinding(ButtonBoxButtons.LeftTop);
    // }
    // public Trigger LeftBottom() {
    //     return buttonBinding(ButtonBoxButtons.LeftBottom);
    // }
    // public Trigger MiddleTop() {
    //     return buttonBinding(ButtonBoxButtons.MiddleTop);
    // }
    // public Trigger MiddleMid() {
    //     return buttonBinding(ButtonBoxButtons.MiddleMid);
    // }
    // public Trigger MiddleBottom() {
    //     return buttonBinding(ButtonBoxButtons.MiddleBottom);
    // }
    // public Trigger PosSwitch() {
    //     return buttonBinding(ButtonBoxButtons.PosSwitch);
    // }


    
}
