class Scheduler {
public:
    typedef void (*TaskCallback)();   // Define a type for task callback

    // Structure for holding task information
    struct Task {
        TaskCallback callback;       // The function to be executed
        unsigned long cycleTime;     // Task cycle time in microseconds
        unsigned long lastExecuted;  // Last execution time in microseconds
        unsigned long execTime;      // Last execution duration in microseconds
    };

    Scheduler() : taskCount(0) {}

    // Add a new task to the scheduler
    void addTask(TaskCallback callback, unsigned long cycleTime) {
        if (taskCount < MAX_TASKS) {
            tasks[taskCount].callback = callback;
            tasks[taskCount].cycleTime = cycleTime;
            tasks[taskCount].lastExecuted = 0;
            tasks[taskCount].execTime = 0;
            taskCount++;
        }
    }

    // Main loop for the scheduler, checks and runs tasks
    void run() {
        unsigned long currentMicros = micros();

        for (int i = 0; i < taskCount; i++) {
            if (currentMicros - tasks[i].lastExecuted >= tasks[i].cycleTime) {
                unsigned long startExecTime = micros();
                tasks[i].callback();  // Execute the task
                unsigned long endExecTime = micros();

                tasks[i].execTime = endExecTime - startExecTime;  // Record execution time
                tasks[i].lastExecuted = currentMicros;            // Update last executed time
            }
        }
    }

    // Get the execution time of a task
    unsigned long getExecTime(TaskCallback callback) {
        for (int i = 0; i < taskCount; i++) {
            if (tasks[i].callback == callback) {
                return tasks[i].execTime;
            }
        }
        return 0;  // Task not found
    }

private:
    static const int MAX_TASKS = 10;  // Adjust as needed
    Task tasks[MAX_TASKS];
    int taskCount;
};
