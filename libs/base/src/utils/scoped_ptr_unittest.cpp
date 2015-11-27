
#include "mvg/utils/scoped_ptr.h"
#include "testing.h"

namespace mvg {
	namespace utils{

		struct FreeMe {
			FreeMe(int *freed) : freed(freed) {}
			~FreeMe() { (*freed)++; }
			int *freed;
		};

		TEST(ScopedPtr, NullDoesNothing) {
			scoped_ptr<FreeMe> x(NULL);
			// Does nothing.
		}

		TEST(ScopedPtr, FreesWhenOutOfScope) {
			int frees = 0;
			{
				scoped_ptr<FreeMe> scoped(new FreeMe(&frees));
				EXPECT_EQ(0, frees);
			}
			EXPECT_EQ(1, frees);
		}

		TEST(ScopedPtr, Operators) {
			int tag = 123;
			scoped_ptr<FreeMe> scoped(new FreeMe(&tag));
			EXPECT_EQ(123, *(scoped->freed));
			EXPECT_EQ(123, *((*scoped).freed));
		}

		TEST(ScopedPtr, Reset) {
			int frees = 0;
			scoped_ptr<FreeMe> scoped(new FreeMe(&frees));
			EXPECT_EQ(0, frees);
			scoped.reset(new FreeMe(&frees));
			EXPECT_EQ(1, frees);
		}

		TEST(ScopedPtr, ReleaseAndGet) {
			int frees = 0;
			FreeMe *allocated = new FreeMe(&frees);
			FreeMe *released = NULL;
			{
				scoped_ptr<FreeMe> scoped(allocated);
				EXPECT_EQ(0, frees);
				EXPECT_EQ(allocated, scoped.get());
				released = scoped.release();
				EXPECT_EQ(0, frees);
				EXPECT_EQ(released, allocated);
			}
			EXPECT_EQ(0, frees);
			delete released;
		}

	}  // namespace
}  // namespace mvg
