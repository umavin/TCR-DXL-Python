list_numbers = [3, 1, 2, 3, 3, 4, 5, 6, 3, 7, 8, 9, 10]
dxl_indexes = [0,4,2]
print(list_numbers.index(dxl_indexes))


'''
import keyboard

while True:
    # Wait for the next event.
    event = keyboard.read_event()
    if event.event_type == keyboard.KEY_DOWN and event.name == 'e':
        print('Robot extending')
        
    elif event.event_type == keyboard.KEY_DOWN and event.name == 'r':
        print('Robot retracting')
        
'''