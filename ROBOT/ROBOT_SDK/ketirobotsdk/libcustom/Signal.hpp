//#ifndef SIGNAL_HPP

//#define SIGNAL_HPP

//#include <functional>

//#include <map>
//#include <iostream>

//// A signal object may call multiple slots with the

//// same signature. You can connect functions to the signal

//// which will be called when the emit() method on the

//// signal object is invoked. Any argument passed to emit()

//// will be passed to the given functions.

// template <typename... Args>

// class Signal {

// public:

//     Signal() : current_id_(0) {}

//     // copy creates new signal

//     Signal(Signal const& other) : current_id_(0) {}

//     // connects a member function to this Signal

//     template <typename T>

//     int connect_member(T *inst, void (T::*func)(Args...)) {

//         return connection([=](Args... args) {

//             (inst->*func)(args...);

//         });

//     }

//     // connects a const member function to this Signal

//     template <typename T>

//     int connect_member(T *inst, void (T::*func)(Args...) const) {

//         return connection([=](Args... args) {

//             (inst->*func)(args...);

//         });

//     }

////     template <typename T>
////     int connect_member(T *inst, void (T::*func)(Args...) ) {

////         return connection([=](Args... args) {

////             (inst->*func)(args...);

////         });

////     }
//     // connects a std::function to the signal. The returned

//     // value can be used to disconnect the function again

//     int connection(std::function<void(Args...)> const& slot) const {

//         // slots_.insert(std::make_pair(++current_id_, slot));
//         slots_.emplace(++current_id_, slot);

//         return current_id_;

//     }

//     // disconnects a previously connected function

//     void disconnect(int id) const {

//         slots_.erase(id);

//     }

//     // disconnects all previously connected functions

//     void disconnect_all() const {

//         slots_.clear();

//     }

//     // calls all connected functions

//     void run(Args... p) {

//         for(auto it : slots_) {

//             it.second(p...);

//         }

//     }

//     // void run(){
//     //     for(auto it=slots_.begin();slots_.end();it++){
//     //         it->second;
//     //     }
//     // }


//     // assignment creates new Signal

//     Signal& operator=(const& other) {

//         disconnect_all();

//     }

// private:

//     // mutable std::map<int, std::function<void(Args...)>> slots_;

////      mutable int current_id_;
//     std::map<int, std::function<void(Args...)>> slots_;

//     int current_id_;
//};


////template <typename... Args>
////class Signal {
////public:
////    Signal() : current_id_(0) {}

////    // 복사 및 대입 금지
////    Signal(const Signal&) = delete;
////    Signal& operator=(const Signal&) = delete;

////    // 멤버 함수 연결 (non-const)
////    template <typename T>
////    int connect_member(T* inst, void (T::*func)(Args...)) {
////        return connect([=](Args... args) {
////            (inst->*func)(args...);
////        });
////    }

////    // 멤버 함수 연결 (const)
////    template <typename T>
////    int connect_member(T* inst, void (T::*func)(Args...) const) {
////        return connect([=](Args... args) {
////            (inst->*func)(args...);
////        });
////    }

////    // 일반 함수/람다 연결
////    int connect(const std::function<void(Args...)>& slot) {
////        slots_.emplace(++current_id_, slot);
////        return current_id_;
////    }

////    // 연결 해제
////    void disconnect(int id) {
////        slots_.erase(id);
////    }

////    // 전체 연결 해제
////    void disconnect_all() {
////        slots_.clear();
////        current_id_=0;
////    }

////    // 모든 슬롯 실행
////    // void emit(Args... args) {
////    //     for (const auto& pair : slots_) {
////    //         pair.second(args...);
////    //     }
////    // }

////    void run(Args... args) {

////        for (const auto& [id, func] : slots_) {
////            func(args...);
////            std::cout<<"Timer Run : " << current_id_<<std::endl;
////        }
////    }
////private:
////    std::map<int, std::function<void(Args...)>> slots_;
////    int current_id_;
////};
//#endif /* SIGNAL_HPP */

#ifndef SIGNAL_HPP

#define SIGNAL_HPP

#include <functional>

#include <map>
#include <iostream>
// A signal object may call multiple slots with the

// same signature. You can connect functions to the signal

// which will be called when the emit() method on the

// signal object is invoked. Any argument passed to emit()

// will be passed to the given functions.

template <typename... Args>

class Signal {

public:

    Signal() : current_id_(0) {}

    // copy creates new signal

    Signal(Signal const& other) : current_id_(0) {}

    // connects a member function to this Signal

    template <typename T>

    int connect_member(T *inst, void (T::*func)(Args...)) {

        return connection([=](Args... args) {

            (inst->*func)(args...);

        });

    }

    // connects a const member function to this Signal

    template <typename T>

    int connect_member(T *inst, void (T::*func)(Args...) const) {

        return connection([=](Args... args) {

            (inst->*func)(args...);

        });

    }

    // connects a std::function to the signal. The returned

    // value can be used to disconnect the function again

    int connection(std::function<void(Args...)> const& slot) const {

        slots_.insert(std::make_pair(++current_id_, slot));

        return current_id_;

    }

    // disconnects a previously connected function

    void disconnect(int id) const {

        slots_.erase(id);

    }

    // disconnects all previously connected functions

    void disconnect_all() const {

        slots_.clear();
        // std::cout<<slots_.size()<<std::endl;
        // std::cout<<&slots_<<std::endl;
        current_id_=0;

    }

    // calls all connected functions

    void run(Args... p) {

        for(auto it : slots_) {

            it.second(p...);
            // std::cout<<current_id_<<std::endl;

        }

    }

    // assignment creates new Signal

    Signal& operator=(Signal const& other) {

        disconnect_all();

    }

private:

//    mutable std::map<int, std::function<void(Args...)>> slots_;

//    mutable int current_id_;

    mutable std::map<int, std::function<void(Args...)>> slots_;

    mutable int current_id_;

};
#endif /* SIGNAL_HPP */


