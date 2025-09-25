import cv2 as cv
import time

#declare grid size
num_rows = 7
num_cols = 7
background = None

def handle_capture():
    global background 
    current_time = int(time.time())
    capture = cv.VideoCapture(1) #needs to be changed to external webcam for

    #get camera resolution
    width  = int(capture.get(cv.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv.CAP_PROP_FRAME_HEIGHT))

    fourcc = cv.VideoWriter_fourcc(*'mp4v') # required 4 digt char code
    output = cv.VideoWriter(f"/Users/Griffinpolly/downloads/intelligent_drone_swarm/output_videos/{current_time}.mp4", fourcc, 20.0, (width, height)) #output file and size specs

    if not capture.isOpened():
        print("cannot open camera")
        exit()

    # make display window resizable
    cv.namedWindow('frame', cv.WINDOW_NORMAL)

    while True:
        #start frame by frame capture
        ret, frame = capture.read()

        #if we are not able to read a frame correctly
        if not ret:
            print("can't receive frame. Exiting...")
            break
        
        display_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        grid_state = detect_objects_in_grid(display_frame)
        draw_grid(display_frame, grid_state)
        
        #convert back to color for display and write to file
        display_frame_color = cv.cvtColor(display_frame, cv.COLOR_GRAY2BGR)
        cv.imshow('frame', display_frame_color)
        output.write(display_frame_color)
        
        #press q to terminate loop
        key = cv.waitKey(1)
        if key == ord('q'):
            print("exiting")
            break
        #press s to set background
        elif key == ord('s'):
            if background is None:
                background = display_frame.copy()
                print("background captured, begin setting objects")
            else:
                print("background has been captured for this run already")

    #release resourses after loop terminatess
    capture.release()
    output.release()
    cv.destroyAllWindows()

def draw_grid(frame, grid_state):
    height, width = frame.shape

    #draw vertical lines
    for i in range(1, num_cols):
        x = int(width * i / num_cols)
        cv.line(frame, (x,0), (x, height), 255, 1)
    
    #draw horizontal lines
    for i in range(1, num_rows):
        y = int(height * i / num_rows)
        cv.line(frame, (0,y), (width, y), 255, 1)
    
    #draw numbers in each grid cell
    for row in range(num_rows):
        for col in range(num_cols):
            x = int((col + 0.5) * width / num_cols)
            y = int((row + 0.5) * height / num_rows)

            state = grid_state[row][col]
            cv.putText(frame, str(state), (x-10, y+10), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

def detect_objects_in_grid(frame):
    global background
    height, width = frame.shape

    if background is None:
        return [[0 for _ in range(num_cols)] for _ in range(num_rows)] #starting with all 0's
    
    grid_state = [[0 for _ in range(num_cols)] for _ in range(num_rows)]
    _, thresh = cv.threshold(frame, 100, 255, cv.THRESH_BINARY_INV)

    # Check each grid cell for changes
    for row in range(num_rows):
        for col in range(num_cols):
            y1 = int(height * row / num_rows)
            y2 = int(height * (row + 1) / num_rows)
            x1 = int(width * col / num_cols)
            x2 = int(width * (col + 1) / num_cols)
            
            # Count white pixels in cell
            cell = thresh[y1:y2, x1:x2]
            white_pixels = cv.countNonZero(cell)
            
            #mark as 1
            cell_area = (y2-y1) * (x2-x1)
            if white_pixels > max(500, cell_area * 0.1): #adjust as needed
                grid_state[row][col] = 1

    return grid_state
    
handle_capture()