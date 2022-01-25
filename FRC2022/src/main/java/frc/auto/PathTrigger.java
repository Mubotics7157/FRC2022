package frc.auto;

public class PathTrigger implements Comparable<PathTrigger> {
		private double percentage;
		private AutoCommand command;

		// Should not be blocking nerds
		public PathTrigger(AutoCommand command, double percentage) {
			this.command = command;
			this.percentage = percentage;
		}

		public static PathTrigger create(AutoCommand c, double p) {
			return new PathTrigger(c, p);
		}
		
		/**
		 * @return the percentage
		 */
		public double getPercentage() {
			return percentage;
		}

		public void playTrigger() {
			command.run();
		}

		@Override
		public int compareTo(PathTrigger other) {
			if (this.percentage > other.percentage)
				return 1;
			if (this.percentage < other.percentage)
				return -1;
			return -0;
		}
	}
