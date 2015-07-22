#ifndef FBLIB_UTILS_DELETE_HANDLER_H_
#define FBLIB_UTILS_DELETE_HANDLER_H_

#include <fblib/utils/referenced.h>

#include <list>

namespace fblib {
	namespace utils{

		/** Class for overriding the default delete behaviour so that users can implement their own object
		  * deletion schemes.
		  * This might be used to implement a protection scheme that avoids
		  * multiple threads deleting objects unintentionally.
		  * Note, the DeleteHandler cannot itself be reference counted, otherwise it
		  * would be responsible for deleting itself!
		  * A static auto_ptr<> is used internally in Referenced.cpp to manage the
		  * DeleteHandler's memory.*/
		class BASE_IMPEXP DeleteHandler
		{
		public:

			typedef std::pair<unsigned int, const fblib::utils::Referenced*> FrameNumberObjectPair;
			typedef std::list<FrameNumberObjectPair> ObjectsToDeleteList;

			DeleteHandler(int numberOfFramesToRetainObjects = 0);

			virtual ~DeleteHandler();

			/** Set the number of frames to retain objects that have been requested for deletion.
			  * When set to zero objects are deleted immediately, by setting to 1 they are kept around for an extra frame etc.
			  * The ability to retain objects for several frames is useful to prevent premature deletion when objects
			  * are still being used by graphics threads that use double buffering of rendering data structures with
			  * non ref_ptr<> pointers to scene graph elements.*/
			void setNumFramesToRetainObjects(unsigned int numberOfFramesToRetainObjects) { _numFramesToRetainObjects = numberOfFramesToRetainObjects; }

			unsigned int getNumFramesToRetainObjects() const { return _numFramesToRetainObjects; }

			/** Set the current frame number so that subsequent deletes get tagged as associated with this frame.*/
			void setFrameNumber(unsigned int frameNumber) { _currentFrameNumber = frameNumber; }

			/** Get the current frame number.*/
			unsigned int getFrameNumber() const { return _currentFrameNumber; }

			inline void doDelete(const fblib::utils::Referenced* object) { delete object; }

			/** Flush objects that are ready to be fully deleted.*/
			virtual void flush();

			/** Flush all objects that the DeleteHandler holds.
			  * Note, this should only be called if there are no threads running with non ref_ptr<> pointers, such as graphics threads.*/
			virtual void flushAll();

			/** Request the deletion of an object.
			  * Depending on users implementation of DeleteHandler, the delete of the object may occur
			  * straight away or be delayed until doDelete is called.
			  * The default implementation does a delete straight away.*/
			virtual void requestDelete(const fblib::utils::Referenced* object);

		protected:

			DeleteHandler(const DeleteHandler&) :
				_numFramesToRetainObjects(0),
				_currentFrameNumber(0) {}
			DeleteHandler operator = (const DeleteHandler&) { return *this; }

			unsigned int            _numFramesToRetainObjects;
			unsigned int            _currentFrameNumber;
			fblib::threads::Mutex      _mutex;
			ObjectsToDeleteList     _objectsToDelete;

		};
	}
}

#endif // FBLIB_UTILS_DELETE_HANDLER_H_
