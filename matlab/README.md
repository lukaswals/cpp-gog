# Matlab Script

This script is used to include the tracker in the UA-DETRAC toolkit. 

## Add cppGOG in UA-DETRAC

1. Create a directory for the tracker inside <detrac-toolkit-path>/trackers. Suppose we name it "CPPGOG"
2. Copy the script "run_tracker.m" into the new created directory
3. Modify the matlab script "DETRAC_experiment.m" to add the tracker. Adding the following line
```
tracker.trackerName = 'CPPGOG'; % This should be the same as the directory created in Step 1
```
4. Run a test!