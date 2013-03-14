/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef ROS_PUBQUEUE_H
#define ROS_PUBQUEUE_H

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <deque>
#include <list>
#include <vector>

#include <ros/ros.h>


template<class T>
class PubMessagePair
{
  public:
    T msg_;
    ros::Publisher pub_;
    PubMessagePair(T& msg, ros::Publisher& pub) :
      msg_(msg), pub_(pub) {}
};

template<class T>
class PubQueue
{
  private:
    boost::shared_ptr<std::deque<boost::shared_ptr<PubMessagePair<T> > > > queue_;
    boost::shared_ptr<boost::mutex> queue_lock_;
    boost::function<void()> notify_func_;

  public:
    typedef boost::shared_ptr<PubQueue<T> > Ptr;

    /* copy constructor
    PubQueue(const PubQueue<T> &_pq)
    {
      *this = _pq;
    } */

    PubQueue(boost::shared_ptr<std::deque<boost::shared_ptr<PubMessagePair<T> > > > queue, 
             boost::shared_ptr<boost::mutex> queue_lock,
             boost::function<void()> notify_func) :
      queue_(queue), queue_lock_(queue_lock), notify_func_(notify_func) {}
    ~PubQueue() {}

    void push(T& msg, ros::Publisher& pub)
    {
      boost::shared_ptr<PubMessagePair<T> > el(new PubMessagePair<T>(msg, pub));
      boost::mutex::scoped_lock lock(*queue_lock_);
      queue_->push_back(el);
      notify_func_();
    }

    void pop(std::vector<boost::shared_ptr<PubMessagePair<T> > >& els)
    {
      boost::mutex::scoped_lock lock(*queue_lock_);
      while(!queue_->empty())
      {
        els.push_back(queue_->front());
        queue_->pop_front();
      }
    }
};

class PubMultiQueue
{
  private:
    std::list<boost::function<void()> > service_funcs_;
    boost::mutex service_funcs_lock_;
    boost::thread service_thread_;
    boost::condition_variable service_cond_var_;
    boost::mutex service_cond_var_lock_;

    template <class T> 
    void serviceFunc(boost::shared_ptr<PubQueue<T> > pq)
    {
      std::vector<boost::shared_ptr<PubMessagePair<T> > > els;
      pq->pop(els);
      for(typename std::vector<boost::shared_ptr<PubMessagePair<T> > >::iterator it = els.begin();
          it != els.end();
          ++it)
      {
        (*it)->pub_.publish((*it)->msg_);
      }
    }
  
  public:
    PubMultiQueue() {}
    ~PubMultiQueue() 
    {
      if(service_thread_.joinable())
      {
        notifyServiceThread();
        service_thread_.join();
      }
    }

    template <class T>
    boost::shared_ptr<PubQueue<T> > addPub()
    {
      boost::shared_ptr<std::deque<boost::shared_ptr<PubMessagePair<T> > > > queue(new std::deque<boost::shared_ptr<PubMessagePair<T> > >);
      boost::shared_ptr<boost::mutex> queue_lock(new boost::mutex);
      boost::shared_ptr<PubQueue<T> > pq(new PubQueue<T>(queue, queue_lock, boost::bind(&PubMultiQueue::notifyServiceThread, this)));
      boost::function<void()> f = boost::bind(&PubMultiQueue::serviceFunc<T>, this, pq);
      {
        boost::mutex::scoped_lock lock(service_funcs_lock_);
        service_funcs_.push_back(f);
      }
      return pq;
    }

    void spinOnce()
    {
      boost::mutex::scoped_lock lock(service_funcs_lock_);
      for(std::list<boost::function<void()> >::iterator it = service_funcs_.begin();
          it != service_funcs_.end();
          ++it)
      {
        (*it)();
      }
    }

    void spin()
    {
      while(ros::ok())
      {
        boost::unique_lock<boost::mutex> lock(service_cond_var_lock_);
        service_cond_var_.wait(lock);
        spinOnce();
      }
    }

    void startServiceThread()
    {
      service_thread_ = boost::thread(boost::bind(&PubMultiQueue::spin, this));
    }

    void notifyServiceThread()
    {
      service_cond_var_.notify_one();
    }
};

#endif
