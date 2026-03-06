#!/usr/bin/env python3
import lgpio
import time
import sys

# --- PIN MAPPING (Matches your viam_driver.py) ---
L_ENC = 19 
R_ENC = 26 

class EncoderCalibrator:
    def __init__(self):
        self.l_ticks = 0
        self.r_ticks = 0
        
        try:
            self.h = lgpio.gpiochip_open(0)
            lgpio.gpio_claim_input(self.h, L_ENC)
            lgpio.gpio_claim_input(self.h, R_ENC)
            
            # Use BOTH_EDGES to match the hardware bridge logic
            lgpio.gpio_claim_alert(self.h, L_ENC, lgpio.BOTH_EDGES)
            lgpio.gpio_claim_alert(self.h, R_ENC, lgpio.BOTH_EDGES)
            
            lgpio.callback(self.h, L_ENC, lgpio.BOTH_EDGES, self.l_cb)
            lgpio.callback(self.h, R_ENC, lgpio.BOTH_EDGES, self.r_cb)
            
            print("--- Encoder Calibration Tool ---")
            print("1. Lift wheels off the ground.")
            print("2. Mark a starting point on the wheel.")
            print("3. Rotate the wheel exactly 10 times.")
            print("4. Divide the result by 10 to get your TPR.\n")
            print("Press Ctrl+C to stop.\n")
            
        except Exception as e:
            print(f"Error: {e}")
            sys.exit(1)

    def l_cb(self, chip, gpio, level, tick):
        self.l_ticks += 1

    def r_cb(self, chip, gpio, level, tick):
        self.r_ticks += 1

    def run(self):
        try:
            while True:
                # Clear line and update counts
                sys.stdout.write(f"\rLEFT: {self.l_ticks:6} | RIGHT: {self.r_ticks:6}")
                sys.stdout.flush()
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n\nFinal Counts:")
            print(f"Left Wheel:  {self.l_ticks}")
            print(f"Right Wheel: {self.r_ticks}")
            print("\nRecommended TPR (if you did 10 turns):")
            print(f"Left:  {self.l_ticks / 10.0}")
            print(f"Right: {self.r_ticks / 10.0}")

if __name__ == "__main__":
    cal = EncoderCalibrator()
    cal.run()
