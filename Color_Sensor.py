import pigpio
import time

s2 = 15
s3 = 14
signal = 18
NUM_CYCLES = 1

def setup(pi):
  pi.set_mode(s2, pigpio.OUTPUT)
  pi.set_mode(s3, pigpio.OUTPUT)
  pi.set_mode(signal, pigpio.INPUT)
  pi.set_pull_up_down(signal, pigpio.PUD_UP)
  print("\n")

def loop(pi):
  temp = 1
  while True:
    pi.write(s2, 0)
    pi.write(s3, 0)
    time.sleep(0.3)
    start = time.time()
    for impulse_count in range(NUM_CYCLES):
      pi.wait_for_edge(signal, pigpio.FALLING_EDGE, 1000)
    duration = time.time() - start
    red = NUM_CYCLES / duration
    print("red value - ", red)

    pi.write(s2, 0)
    pi.write(s3, 1)
    time.sleep(0.3)
    start = time.time()
    for impulse_count in range(NUM_CYCLES):
      pi.wait_for_edge(signal, pigpio.FALLING_EDGE, 1000)
    duration = time.time() - start
    blue = NUM_CYCLES / duration
    print("blue value - ", blue)

    pi.write(s2, 1)
    pi.write(s3, 1)
    time.sleep(0.3)
    start = time.time()
    for impulse_count in range(NUM_CYCLES):
      pi.wait_for_edge(signal, pigpio.FALLING_EDGE, 1000)
    duration = time.time() - start
    green = NUM_CYCLES / duration
    print("green value - ", green)
    #time.sleep(2)

def endprogram(pi):
  pi.stop()

if __name__ == '__main__':
  pi = pigpio.pi()
  setup(pi)

  try:
    loop(pi)
  except KeyboardInterrupt:
    endprogram(pi)
