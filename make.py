# -*- coding: utf-8 -*-
from jinja2 import Environment, FileSystemLoader
import os
TEMPLATE = os.path.dirname(os.path.abspath(__file__)) + '/template/'

def generate_index_html():

	index = open("index.html","w")

	profile = unicode(open(TEMPLATE+"profile.html","r").read(),encoding='utf-8')
	works = unicode(open(TEMPLATE+"works.html","r").read(),encoding='utf-8')
	studies = unicode(open(TEMPLATE+"studies.html","r").read(),encoding='utf-8')
	blog = unicode(open(TEMPLATE+"blog.html","r").read(),encoding='utf-8')
	contact = unicode(open(TEMPLATE+"contact.html","r").read(),encoding='utf-8')

	env = Environment(loader=FileSystemLoader(TEMPLATE, encoding='utf_8'))
	tpl = env.get_template('index.html')

	html_string = tpl.render({'profile': profile,
							'works': works,
							'studies': studies,
							'blog' : blog,
							'contact': contact }).encode('utf-8')
	index.write(html_string)
	print "Generated index.html"

if __name__ == '__main__':
	generate_index_html()