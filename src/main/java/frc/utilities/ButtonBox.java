package frc.utilities;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonBox extends CommandGenericHID {

    public static enum ButtonBoxButtons {
        C1(1),
        C2(2),
        C3(3),
        C4(4),
        C5(5),
        
        U1(17),
        U2(18),
        U3(19),
    
        AH(14),
        AL(15),
        AP(16),

        SW1(6),

        R4L(7),
        R4R(8),
        R3L(9),
        R3R(10),
        R2L(11),
        R2R(12),
        R1(13)

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
