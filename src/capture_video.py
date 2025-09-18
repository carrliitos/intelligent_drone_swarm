import cv2 as cv

num_rows = 7
num_cols = 7

def handle_capture():
    capture = cv.VideoCapture(1)
    fourcc = cv.VideoWriter_fourcc(*'mp4v')
    output = cv.VideoWriter('/Users/Griffinpolly/downloads/output.mp4', fourcc, 20.0, (640, 480)) #output file and size specs

    if not capture.isOpened():
        print("cannot open camera")
        exit()
    while True:
        #start frame by frame capture
        ret, frame = capture.read()

        #if we are not able to read a frame correctly
        if not ret:
            print("can't receive frame. Exiting...")
            break

        frame_resized = cv.resize(frame, (640, 480))
        grid_state = detect_objects_in_grid(frame_resized)
        draw_grid(frame_resized, grid_state)
        output.write(frame_resized)
        
        #display
        cv.imshow('frame', frame_resized)
        
        #press q to terminate
        if cv.waitKey(1) == ord('q'):
            break

    #release resources after loop terminatess
    capture.release()
    output.release()
    cv.destroyAllWindows()

def draw_grid(frame, grid_state):
    height, width, _ = frame.shape

    #draw vertical lines
    for i in range(1, num_cols):
        x = int(width * i / num_cols)
        cv.line(frame, (x,0), (x, height), (0, 255, 0), 1)
    
    #draw horizontal lines
    for i in range(1, num_rows):
        y = int(height * i / num_rows)
        cv.line(frame, (0,y), (width, y), (0, 255, 0), 1)
    
    #draw numbers in each grid cell
    for row in range(num_rows):
        for col in range(num_cols):
            x = int((col + 0.5) * width / num_cols)
            y = int((row + 0.5) * height / num_rows)

            state = grid_state[row][col]
            cv.putText(frame, str(state), (x-10, y+10),cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

def detect_objects_in_grid(frame):
    grid_state = [[0 for _ in range(num_cols)] for _ in range(num_rows)]
    
    # object detection here

    return grid_state
    
handle_capture()