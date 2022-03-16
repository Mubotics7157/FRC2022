// Copyright 2019 FRC Team 3476 Code Orange

package frc.util.Threading;

import java.time.Duration;
import java.util.Vector;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.locks.LockSupport;

/**
 * Keeps track of scheduled tasks and checks every 5ms to execute the task with
 * given threads. Tasks that have not finished in the given time will not be ran
 * again until it finishes.
 */
// TODO: Add remove function
public class ThreadScheduler implements Runnable {
	
	private Vector<Schedule> schedules;
	private volatile boolean isRunning;
	private volatile boolean paused;

	public ThreadScheduler() {
		schedules = new Vector<Schedule>();
		isRunning = true;
		paused = true;
		ExecutorService schedulingThread = Executors.newSingleThreadExecutor();
		schedulingThread.execute(this);
	}

	@Override
	public void run() {
		long waitTime = Duration.ofMillis(5).toNanos();
		while (isRunning) {
			if (!paused) {
				synchronized (this) {
					for (Schedule schedule : schedules) {
						/*if (schedule.removeNextFrame) {
							safeRemove(schedule);
						} else {*/
							schedule.executeIfReady();
						//}
					}
				}
			}
			LockSupport.parkNanos(waitTime);
		}
	}

	public void schedule(Threaded task, ExecutorService thread) {
		schedules.add(new Schedule(task, thread));
	}

	public void pause() {
		paused = true;
	}

	public void resume() {
		paused = false;
	}

	public void shutdown() {
		isRunning = false;
	}

	/*public void remove(Schedule schedule) {
		schedule.removeNextFrame = true;
	}

	private void safeRemove(Schedule schedule) {
		schedules.remove(schedule);
	}*/

	private static class Schedule {
		Threaded task;
		public long taskTime;
		//boolean removeNextFrame;
		ExecutorService thread;

		private Schedule(Threaded task, ExecutorService thread) {
			this.task = task;
			this.thread = thread;
			taskTime = System.nanoTime();
		}

		public void executeIfReady() {
			if (task.isUpdated()) {
				if (System.nanoTime() - taskTime > task.getPeriod()) {
					thread.submit(task);
					taskTime = System.nanoTime();
				}
			}
		}
	}
}