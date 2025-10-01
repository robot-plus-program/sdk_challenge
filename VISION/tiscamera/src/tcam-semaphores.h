
#ifndef TCAM_SEMAPHORES_H
#define TCAM_SEMAPHORES_H

#include <stdexcept>
#include <sys/ipc.h> /* general SysV IPC structures          */
#include <sys/sem.h>
#include <sys/types.h> /* various type definitions.            */
#include <utility>

namespace tcam
{


/**
 * @return id of new semaphore; -1 on error, check errno
 */
inline int semaphore_create(const key_t sem_id)
{

    /* create a semaphore set with ID 250, with one semaphore   */
    /* in it, with access only to the owner.                    */
    int sem_set_id = semget(sem_id, 1, IPC_CREAT | 0666);

    if (sem_set_id == -1)
    {
        return -1;
    }

    union semun
    { /* semaphore value, for semctl().     */
        int val;
        struct semid_ds* buf;
        ushort* array;
    } sem_val;

    /* initialize the first (and single) semaphore in our set to '1'. */
    sem_val.val = 1;
    int rc = semctl(sem_set_id, 0, SETVAL, sem_val);
    if (rc == -1)
    {
        return -1;
    }

    return sem_set_id;
}


inline void semaphore_destroy(int sem_id)
{

    semctl(sem_id, 0, IPC_RMID);
}


/*
 * function: sem_lock. locks the semaphore, for exclusive access to a resource.
 * input:    semaphore set ID.
 * output:   none.
 */
inline void semaphore_lock(int sem_set_id)
{
    /* structure for semaphore operations.   */
    struct sembuf sem_op;

    /* wait on the semaphore, unless it's value is non-negative. */
    sem_op.sem_num = 0;
    sem_op.sem_op = -1;
    sem_op.sem_flg = 0;
    semop(sem_set_id, &sem_op, 1);
}

/*
 * function: sem_unlock. un-locks the semaphore.
 * input:    semaphore set ID.
 * output:   none.
 */
inline void semaphore_unlock(int sem_set_id)
{
    /* structure for semaphore operations.   */
    struct sembuf sem_op;

    /* signal the semaphore - increase its value by one. */
    sem_op.sem_num = 0;
    sem_op.sem_op = 1; /* <-- Comment 3 */
    sem_op.sem_flg = 0;
    semop(sem_set_id, &sem_op, 1);
}

class semaphore
{
public:
    semaphore() = default;

    semaphore(const semaphore&) = delete;
    semaphore& operator=(const semaphore&) = delete;

    semaphore(semaphore&& op2) noexcept
    {
        sem_id_ = op2.sem_id_;
        op2.sem_id_ = 0;
    }
    semaphore& operator=(semaphore&& op2) noexcept
    {
        destroy();
        std::swap(sem_id_, op2.sem_id_);
        return *this;
    }
    ~semaphore()
    {
        if (sem_id_)
        {
            semaphore_destroy(sem_id_);
        }
    }

    void lock() noexcept
    {
        semaphore_lock(sem_id_);
    }
    void unlock() noexcept
    {
        semaphore_unlock(sem_id_);
    }

    // Note: Throws on failure
    static semaphore create(const key_t key)
    {
        auto id = semaphore_create(key);
        if (id == -1)
        {
            throw std::runtime_error("Failed to create semaphore");
        }
        return semaphore(id);
    }

private:
    explicit semaphore(int id) noexcept : sem_id_(id) {}

    void destroy() noexcept
    {
        if (sem_id_)
        {
            semaphore_destroy(sem_id_);
        }
        sem_id_ = 0;
    }

    int sem_id_ = 0;
};

} /* namespace tcam */

#endif /* TCAM_SEMAPHORES_H */
