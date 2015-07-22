#include "base_precomp.h"
#include <fblib/utils/delete_handler.h>
#include <fblib/utils/notify.h>

namespace fblib
{
	namespace utils{
		DeleteHandler::DeleteHandler(int numberOfFramesToRetainObjects) :
			_numFramesToRetainObjects(numberOfFramesToRetainObjects),
			_currentFrameNumber(0)
		{
		}

		DeleteHandler::~DeleteHandler()
		{
			// flushAll();
		}

		void DeleteHandler::flush()
		{
			typedef std::list<const fblib::utils::Referenced*> DeletionList;
			DeletionList deletionList;

			{
				// gather all the objects to delete whilst holding the mutex to the _objectsToDelete
				// list, but delete the objects outside this scoped lock so that if any objects deleted
				// unref their children then no deadlock happens.
				fblib::threads::ScopedLock<fblib::threads::Mutex> lock(_mutex);
				unsigned int frameNumberToClearTo = _currentFrameNumber - _numFramesToRetainObjects;

				ObjectsToDeleteList::iterator itr;
				for (itr = _objectsToDelete.begin();
					itr != _objectsToDelete.end();
					++itr)
				{
					if (itr->first > frameNumberToClearTo) break;

					deletionList.push_back(itr->second);

					itr->second = 0;
				}

				_objectsToDelete.erase(_objectsToDelete.begin(), itr);
			}

			for (DeletionList::iterator ditr = deletionList.begin();
				ditr != deletionList.end();
				++ditr)
			{
				doDelete(*ditr);
			}

		}

		void DeleteHandler::flushAll()
		{
			unsigned int temp_numFramesToRetainObjects = _numFramesToRetainObjects;
			_numFramesToRetainObjects = 0;

			typedef std::list<const fblib::utils::Referenced*> DeletionList;
			DeletionList deletionList;

			{
				// gather all the objects to delete whilst holding the mutex to the _objectsToDelete
				// list, but delete the objects outside this scoped lock so that if any objects deleted
				// unref their children then no deadlock happens.
				fblib::threads::ScopedLock<fblib::threads::Mutex> lock(_mutex);
				ObjectsToDeleteList::iterator itr;
				for (itr = _objectsToDelete.begin();
					itr != _objectsToDelete.end();
					++itr)
				{
					deletionList.push_back(itr->second);
					itr->second = 0;
				}

				_objectsToDelete.erase(_objectsToDelete.begin(), _objectsToDelete.end());
			}

			for (DeletionList::iterator ditr = deletionList.begin();
				ditr != deletionList.end();
				++ditr)
			{
				doDelete(*ditr);
			}

			_numFramesToRetainObjects = temp_numFramesToRetainObjects;
		}

		void DeleteHandler::requestDelete(const fblib::utils::Referenced* object)
		{
			if (_numFramesToRetainObjects == 0) doDelete(object);
			else
			{
				fblib::threads::ScopedLock<fblib::threads::Mutex> lock(_mutex);
				_objectsToDelete.push_back(FrameNumberObjectPair(_currentFrameNumber, object));
			}
		}
	}
} // end of namespace fblib