# -*- coding: utf-8 -*-
from jinja2 import Environment, FileSystemLoader
import os
TEMPLATE_ja = os.path.dirname(os.path.abspath(__file__)) + '/template/ja/'
TEMPLATE_en = os.path.dirname(os.path.abspath(__file__)) + '/template/en/'

def generate_index_html_japanese():

	index = open("index.html","w")

	profile = unicode(open(TEMPLATE_ja+"profile.html","r").read(),encoding='utf-8')
	works = unicode(open(TEMPLATE_ja+"works.html","r").read(),encoding='utf-8')
	studies = unicode(open(TEMPLATE_ja+"studies.html","r").read(),encoding='utf-8')
	contact = unicode(open(TEMPLATE_ja+"contact.html","r").read(),encoding='utf-8')

	env = Environment(loader=FileSystemLoader(TEMPLATE_ja, encoding='utf_8'))
	tpl = env.get_template('index.html')

	html_string = tpl.render({'profile': profile,
							'works': works,
							'studies': studies,
							'contact': contact }).encode('utf-8')
	index.write(html_string)
	print "Generated index.html"

def generate_index_html_english():

	index = open("index_en.html","w")

	profile = unicode(open(TEMPLATE_en+"profile.html","r").read(),encoding='utf-8')
	works = unicode(open(TEMPLATE_en+"works.html","r").read(),encoding='utf-8')
	studies = unicode(open(TEMPLATE_en+"studies.html","r").read(),encoding='utf-8')
	contact = unicode(open(TEMPLATE_en+"contact.html","r").read(),encoding='utf-8')

	env = Environment(loader=FileSystemLoader(TEMPLATE_en, encoding='utf_8'))
	tpl = env.get_template('index.html')

	html_string = tpl.render({'profile': profile,
							'works': works,
							'studies': studies,
							'contact': contact }).encode('utf-8')
	index.write(html_string)
	print "Generated index_en.html"

if __name__ == '__main__':
	generate_index_html_japanese()
	generate_index_html_english()