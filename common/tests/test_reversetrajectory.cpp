#include "../trajectory.hpp"
#include <cstdlib>

static void test_double_reverse();

int main(int argc, char** argv) {
	try {
		test_double_reverse();
		exit(EXIT_SUCCESS);
	} catch (std::runtime_error& e) {
		std::cerr << "Test " << __FILE__ << " failed: " << std::endl
				  << e.what() << std::endl;
		exit(EXIT_FAILURE);
	}
}

void test_double_reverse() {

	Trajectory t;
	srand(1234);
	for (int i = 0; i < 250; ++i) {
		Trajectory::TrajectoryPoint p;
		p.setTime(i*0.1);
		p.setXCoord(rand() % 100 - 50);
		p.setYCoord(rand() % 100 - 50);
		p.setZCoord(rand() % 100 - 50);
		p.setHeading(rand() % 100 - 5*M_PI);
		p.setCurvature(rand() % 100 - 50);
		p.setLongitudinalVelocity(rand() % 100 - 50);
		p.setLongitudinalAcceleration(rand() % 100 - 50);
		p.setLateralVelocity(rand() % 100 - 50);
		p.setLateralAcceleration(rand() % 100 - 50);

		t.points.push_back(p);
	}

	if (t == t.reversed().reversed()) {
		return;
	}
	throw std::runtime_error("Reversing twice does not produce the original trajectory");
}
