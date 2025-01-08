from typing import Callable, Optional, Tuple, Dict, Any
from threading import Thread, Event, Condition

class SuspendableThread(Thread):
    """A thread that can be suspended or killed externally (in a thread-safe way)."""

    def __init__(
        self,
        *,
        setup: Callable[[], None] | None = None,
        target: Callable[[], None] | None = None,
        args: Optional[Tuple[Any, ...]] = (),  # Positional arguments as a tuple
        kwargs: Optional[Dict[str, Any]] = None,  # Keyword arguments as a dictionary
        clean_up: Callable[[], None] | None = None,
        run_in_background: bool = True,
    ):
        super().__init__(daemon=run_in_background)

        self._setup = setup
        self._target = target
        self._args = args if args else ()  # Default to an empty tuple
        self._kwargs = kwargs if kwargs else {}  # Default to an empty dictionary
        self._clean_up = clean_up

        self._should_kill_flag = Event()
        self._should_suspend_flag = Event()
        self._suspend_cond = Condition()
        self._is_alive_flag = Event()
        self._is_running_flag = Event()

        self._is_alive_flag.set()
        self._should_kill_flag.clear()
        self._should_suspend_flag.clear()

    @property
    def is_running(self):
        """
        Whether the thread is currently running.

        Returns
        -------
        `True` if the thread is currently running, `False` otherwise.
        """
        return self._is_running_flag.is_set()

    @property
    def is_alive(self):
        """
        Whether the thread is alive.

        Once a thread has been killed, it cannot be restarted.

        Returns
        -------
        `True` if the thread is alive, `False` otherwise.
        """
        return self._is_alive_flag.is_set()

    def run(self):
        """
        Runs the thread
        """
        self._is_running_flag.set()

        if self._setup is not None:
            self._setup()

        while not self._should_kill_flag.is_set():
            if self._should_suspend_flag.is_set():
                with self._suspend_cond:
                    self._suspend_cond.wait_for(
                        lambda: not self._should_suspend_flag.is_set()
                        or self._should_kill_flag.is_set()
                    )

                if self._should_kill_flag.is_set():
                    break

            if self._target is not None:
                self._target(*self._args, **self._kwargs)

        if self._clean_up is not None:
            self._clean_up()

        self._is_alive_flag.clear()

    def suspend(self):
        """Suspends the thread"""
        if not self._should_suspend_flag.is_set():
            self._is_running_flag.clear()
            self._should_suspend_flag.set()

    def resume(self):
        """Resumes the thread"""
        if self._should_suspend_flag.is_set():
            self._is_running_flag.set()
            self._should_suspend_flag.clear()

    def kill(self):
        """Kills the thread (permanently)"""
        if not self._should_kill_flag.is_set():
            self._should_kill_flag.set()
