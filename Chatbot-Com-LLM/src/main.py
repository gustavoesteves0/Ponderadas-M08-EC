import streamlit as st
import os
from dotenv import load_dotenv
from llama_index.core import SimpleDirectoryReader, VectorStoreIndex, Settings
from llama_index.llms.gemini import Gemini
from llama_index.embeddings.huggingface import HuggingFaceEmbedding
from llama_index.core.node_parser import SentenceSplitter

load_dotenv()

documents = SimpleDirectoryReader(
    input_files=["doc.pdf"]
    ).load_data()

llm = Gemini(api_key=os.getenv('GENERATIVE_AI_KEY'), model="models/gemini-1.5-flash", temperature=0.3, top_p=1, top_k=32)
embed_model = HuggingFaceEmbedding(model_name="BAAI/bge-small-en-v1.5")
text_splitter = SentenceSplitter(chunk_size=1024, chunk_overlap=20)

Settings.llm = llm
Settings.embed_model = embed_model
Settings.text_splitter = text_splitter

index = VectorStoreIndex.from_documents(documents, show_progress=True)
query_engine = index.as_query_engine()

#Streamlit app
if "messages" not in st.session_state:
    st.session_state.messages = []

st.title("Chatbot de Segurança Industrial")

for message in st.session_state.messages:
    with st.chat_message(message["role"]):
        st.markdown(message["content"])

if prompt := st.chat_input("Faça uma pergunta :)"):

    st.chat_message("user").markdown(prompt)
    st.session_state.messages.append({"role": "user", "content": prompt})

    try:

        response = query_engine.query(prompt)
        
        with st.chat_message("assistant"):
            st.markdown(response)
        st.session_state.messages.append({"role": "assistant", "content": response})
    
    except Exception as e:
        st.error(f"An error occurred: {str(e)}")