MODULE CFreeRapid
    CONST num STATE_IDLE:=0;
    CONST num STATE_RUNNING:=1;
    !CONST num STATE_FINISHED:=2;
    PERS num state:=0;
    !---------------------------------------------------------
    ! Program data
    !---------------------------------------------------------
    ! Interrupt numbers.
    LOCAL VAR intnum intnum_run_rapid_routine;

    !---------------------------------------------------------
    ! Primary procedures
    !---------------------------------------------------------
    PROC initializeRAPIDModule()
        ! Setup an interrupt signal.
        IDelete intnum_run_rapid_routine;
        CONNECT intnum_run_rapid_routine WITH handleRunCFREERoutine;
        ISignalDI CFREE_RUN_PATH,HIGH,intnum_run_rapid_routine;
    ENDPROC

    !---------------------------------------------------------
    ! Traps
    !---------------------------------------------------------
    LOCAL TRAP handleRunCFREERoutine
        !TPWrite "in handleRunRAPIDRoutine";
        loadModule "Home:","CfreePath.mod";
        !StopMove;
        StorePath; state:=STATE_RUNNING;
        %"CfreePath:path"%;
        RestoPath;
        !StartMove;
        unloadModule "Home/CfreePath.mod"; state:=STATE_IDLE;

    ENDTRAP

    LOCAL PROC loadModule(string path,string file)
        Load path\File:=file;
    ERROR
        IF ERRNO=ERR_LOADED THEN
            UnLoad path\File:=file;
            RETRY;
        ENDIF
    ENDPROC

    LOCAL PROC unloadModule(string path)
        UnLoad path;
    ERROR
        TPWrite "'runModuleUnload' failed!";
        TRYNEXT;
    ENDPROC

    PROC entry()
        TPErase;
        initializeRAPIDModule;

        TPWrite "Starting loop";
        WHILE TRUE DO
            WaitTime 0.005;
        ENDWHILE

    ENDPROC
ENDMODULE
