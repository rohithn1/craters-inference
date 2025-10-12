import curses
import time
from steering_module import SteeringController


def main(stdscr, steering_obj):
    # Clear screen
    stdscr.clear()

    # Inform the user how to exit
    stdscr.addstr(0, 0, "Press 'a' to print 'left' and 'd' to print 'right'. Press 'q' to exit.")
    stdscr.refresh()

    # Set nodelay to True to make getch non-blocking
    stdscr.nodelay(True)

    a_pressed = False
    d_pressed = False

    prior_action = "straight"
    new_action = False

    while True:
        key = stdscr.getch()

        if key == ord('a'):
            a_pressed = True
            d_pressed = False
            if prior_action != "left":
                prior_action = "left"
                new_action = True
            stdscr.refresh()
        elif key == ord('d'):
            d_pressed = True
            a_pressed = False
            if prior_action != "right":
                prior_action = "right"
                new_action = True
            stdscr.refresh()
        elif key == ord('q'):
            break
        elif key == -1:  # No key pressed
            if a_pressed or d_pressed:
                a_pressed = False
                d_pressed = False
                if prior_action != "straight":
                    prior_action = "straight"
                    new_action = True
                stdscr.refresh()
        time.sleep(0.05)
        if (a_pressed and new_action):
            stdscr.addstr(1, 0, "left    ") 
        elif (d_pressed and new_action):
            stdscr.addstr(1, 0, "right   ")
        elif (not a_pressed and not d_pressed and new_action):
            stdscr.addstr(1, 0, "straight")
        new_action = False

steering_obj = SteeringController(init_sleep_factor=4)
time.sleep(2)

curses.wrapper(main, steering_obj)

del steering_obj
