import time 

def wait(duration):
    print(f"wait for {duration}s")
    for i in range(12):
        print(f"Sleeped for {i*5}s")
        time.sleep(5)
    print('done')

if __name__ == "__main__":
    wait(60)