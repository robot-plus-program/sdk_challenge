try: 
    import logging
    from telegram import Update
    from telegram.ext import Application, CommandHandler, MessageHandler, filters, CallbackContext
    from pyconnect.llm.llama import LLamaClient
    from pyconnect.utils import parse_keys_values
    from pyconnect.sns_bot.utils import handle_msg
except Exception as e:
    print(e)

# Enable logging
logging.basicConfig(
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s", level=logging.INFO
)
logger = logging.getLogger(__name__)

args = parse_keys_values(optional_args={'bot_token':'7859374418:AAGyUHuv2Sc-HWh4-8JCRhRo3MUxjagDWXc', 
                                        'url':'10.252.216.250:11434'})


class ChatBot():
    def __init__(self):
        self.llmobj = LLamaClient(url=args['url'])
        self.app = app = Application.builder().token(args['bot_token']).build()
        # Command handlers
        self.app.add_handler(CommandHandler("start", self.start))

        # Message handler for all text messages
        app.add_handler(MessageHandler(filters.TEXT & ~filters.COMMAND, self.handle_message))

    async def start(self, update: Update, context: CallbackContext) -> None:
        await update.message.reply_text("Hello! Send me a message, and I'll respond using Ollama AI.")

    async def handle_message(self, update: Update, context: CallbackContext) -> None:
        user_message = update.message.text
        chat_id = update.message.chat_id

        self.llmobj,res = handle_msg(llmobj=self.llmobj, msg=user_message, logger=logger)

        await context.bot.send_message(chat_id=chat_id, text=res)

    def listen(self):  
        self.app.run_polling()



# def main():
#     """Starts the bot."""
#     app = Application.builder().token(args['bot_token']).build()

#     # Command handlers
#     app.add_handler(CommandHandler("start", start))

#     # Message handler for all text messages
#     app.add_handler(MessageHandler(filters.TEXT & ~filters.COMMAND, handle_message))

#     # Start polling
#     logger.info("Bot is running...")
#     app.run_polling()
#     # threading.Thread(target=app.run_polling, daemon=False).start()

if __name__ == "__main__":
    chatbot = ChatBot()
    chatbot.listen()
