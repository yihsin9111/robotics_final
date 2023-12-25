import time

def delayMicroseconds(delay_us) -> None:
    start_time = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
    end_time = time.clock_gettime_ns(time.CLOCK_BOOTTIME)

    while(end_time - start_time) < delay_us * 1000:
        end_time = time.clock_gettime_ns(time.CLOCK_BOOTTIME)

def main():
    return

if __name__ == "__main__":
    main()
