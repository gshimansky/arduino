#ifndef MESSAGE_QUEUE_H
#define MESSAGE_QUEUE_H

#include <list>
#include <UniversalTelegramBot.h>

class TelegramMessageQueue {
public:
  TelegramMessageQueue(UniversalTelegramBot &bot) : m_bot(bot)
  {}

  void enqueue(String &&user_id, String &&msg, String &&keyboard_json = "", const bool resize = true) {
    m_list.emplace_back(std::move(user_id), std::move(msg), std::move(keyboard_json), resize);
  }

  void send_one(void) {
    if (m_list.size() > 0) {
      const Message &m = m_list.front();
      bool sent = false;

      if (m.m_keyboard_json == "") {
        sent = m_bot.sendMessage(m.m_user_id, m.m_msg, "");
      } else {
        sent = m_bot.sendMessageWithReplyKeyboard(m.m_user_id, m.m_msg, "", m.m_keyboard_json, m.m_resize);
      }

      if (sent) {
        m_list.pop_front();
      }
    }
  }

private:
  struct Message {
    Message(String &&user_id, String &&msg, String &&keyboard_json, const bool resize) :
      m_user_id(std::move(user_id)), m_msg(std::move(msg)), m_keyboard_json(std::move(keyboard_json)), m_resize(resize)
    {}
    String m_user_id, m_msg, m_keyboard_json;
    bool m_resize;
  };

  std::list<Message> m_list;
  UniversalTelegramBot &m_bot;
};

#endif
