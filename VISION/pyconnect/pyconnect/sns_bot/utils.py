

def handle_msg(llmobj, msg, logger):
    try:
        if msg=='@clean':
            llmobj.reset_history(tag=llmobj.current_tag)
            res = f'History of tag [{llmobj.current_tag} removed]'
        elif msg=='@reset':
            llmobj.reset_history(tag='all')
            res  = 'All histories are removed'
        elif '@tag' in msg:
            llmobj.current_tag = msg.split(':')[-1]
            res = f'Switched to {llmobj.current_tag}'
        elif '@select_model' in msg:
            model_name = msg.split(':')[-1]
            if 'vision' in model_name:
                llmobj.vision_model = model_name
            else:
                llmobj.text_model = model_name

            try:
                res = llmobj.chat(prompt='hi')
            except: 
                res = f'Cannot select model {model_name}'
        else:
            res = llmobj.chat(prompt=msg)
    except Exception as e:
        logger.error(f"Error communicating with Ollama: {e}")
        res = "Oops! There was an error processing your request."

    res = f'[{llmobj.text_model}:{llmobj.current_tag}]: \n{res}'
    return llmobj, res